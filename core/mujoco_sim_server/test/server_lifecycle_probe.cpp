#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include <ecal/ecal.h>
#include <ecal/pubsub/publisher.h>
#include <ecal/pubsub/subscriber.h>
#include <ecal/service/client.h>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>
#include <mujoco_sim/v3/sync.pb.h>

namespace {

using namespace std::chrono_literals;

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

template <typename Request, typename Response>
bool Call(eCAL::CServiceClient* client, const std::string& method,
          const Request& request, Response* response, std::string* error) {
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    eCAL::ServiceResponseVecT responses;
    client->CallWithResponse(method, request.SerializeAsString(), responses,
                             200);
    for (const auto& candidate : responses) {
      if (candidate.call_state != eCAL::eCallState::executed ||
          candidate.ret_state != 0) {
        continue;
      }
      if (!response->ParseFromString(candidate.response)) {
        *error = "invalid service response: " + method;
        return false;
      }
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  *error = "service call timed out: " + method;
  return false;
}

struct ObservedState {
  std::mutex mutex;
  std::condition_variable condition;
  mujoco_sim::v3::ServerStatus status;
  mujoco_sim::protocol::DataFrameHeader tick;
  std::uint64_t status_sequence = 0;
  std::uint64_t tick_sequence = 0;
};

template <typename Predicate>
bool WaitFor(ObservedState* observed, Predicate predicate,
             std::chrono::milliseconds timeout = 5s) {
  std::unique_lock lock(observed->mutex);
  return observed->condition.wait_for(lock, timeout,
                                      [&] { return predicate(*observed); });
}

const mujoco_sim::v3::AdapterStatus* FindAdapter(
    const mujoco_sim::v3::ServerStatus& status, const std::string& id,
    std::uint64_t session_id) {
  for (const auto& controller : status.adapters()) {
    if (controller.adapter_id() == id &&
        controller.session_id() == session_id) {
      return &controller;
    }
  }
  return nullptr;
}

bool PublishUntilLocalError(
    eCAL::CPublisher* publisher, const std::string& payload,
    ObservedState* observed, const std::string& adapter_id,
    std::uint64_t session_id) {
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    (void)publisher->Send(payload);
    if (WaitFor(
            observed,
            [&](const auto& state) {
              const auto* controller =
                  FindAdapter(state.status, adapter_id, session_id);
              return controller != nullptr &&
                     controller->state() == mujoco_sim::v3::ERROR &&
                     state.status.state() == mujoco_sim::v3::RUNNING;
            },
            100ms)) {
      return true;
    }
  }
  return false;
}

mujoco_sim::v3::RegisterAdapterResponse Register(
    eCAL::CServiceClient* client, const std::string&,
    const std::string& adapter_id, std::string* error,
    std::uint32_t timeout_ms = 50,
    const std::string& observation_name = "hinge_pos",
    const std::string& command_name = "hinge_motor") {
  mujoco_sim::v3::RegisterAdapterRequest request;
  request.set_adapter_id(adapter_id);
  request.set_role(mujoco_sim::v3::CONTROLLER);
  request.set_commit_timeout_ms(timeout_ms);
  request.set_session_timeout_ms(std::max<std::uint32_t>(1000, timeout_ms * 2));
  request.add_observation_names(observation_name);
  request.add_command_names(command_name);
  mujoco_sim::v3::RegisterAdapterResponse response;
  if (!Call(client, mujoco_sim::protocol::kRegisterAdapterMethod, request,
            &response, error)) {
    return {};
  }
  return response;
}

bool Activate(eCAL::CServiceClient* client, const std::string&,
              const mujoco_sim::v3::RegisterAdapterResponse& registration,
              std::string* error) {
  mujoco_sim::v3::ActivateAdapterRequest request;
  request.set_session_id(registration.session_id());
  request.set_epoch(registration.epoch());
  mujoco_sim::v3::ActivateAdapterResponse response;
  return Call(client, mujoco_sim::protocol::kActivateAdapterMethod, request,
              &response, error) &&
         response.code() == mujoco_sim::v3::OK;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: server_lifecycle_probe SERVER_NAME\n";
    return 1;
  }
  const std::string server_name = argv[1];
  const std::string adapter_id = "dynamic_adapter";
  const std::string fast_adapter_id = "fast_adapter";
  if (!eCAL::Initialize("mujoco-sim-server-lifecycle-probe")) {
    std::cerr << "failed to initialize eCAL\n";
    return 1;
  }

  int result = 1;
  {
    eCAL::ServiceMethodInformationSetT methods;
    methods.insert(MethodInfo(mujoco_sim::protocol::kRegisterAdapterMethod));
    methods.insert(MethodInfo(mujoco_sim::protocol::kActivateAdapterMethod));
    methods.insert(
        MethodInfo(mujoco_sim::protocol::kUnregisterAdapterMethod));
    eCAL::CServiceClient registry(
        mujoco_sim::protocol::RegistryServiceName(server_name), methods);
    eCAL::CPublisher commit_publisher(
        mujoco_sim::protocol::CommitTopicName(server_name));
    eCAL::CSubscriber status_subscriber(
        mujoco_sim::protocol::StatusTopicName(server_name));
    eCAL::CSubscriber tick_subscriber(
        mujoco_sim::protocol::TickTopicName(server_name, adapter_id));
    eCAL::CSubscriber fast_tick_subscriber(
        mujoco_sim::protocol::TickTopicName(server_name, fast_adapter_id));

    ObservedState observed;
    std::atomic<std::uint64_t> accepted_session{0};
    std::atomic<std::uint64_t> fast_session{0};
    std::atomic<bool> auto_commit{false};
    std::atomic<bool> fast_auto_commit{false};
    std::atomic<std::uint64_t> fast_commit_count{0};
    status_subscriber.SetReceiveCallback(
        [&](const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
            const eCAL::SReceiveCallbackData& data) {
          mujoco_sim::v3::ServerStatus status;
          if (data.buffer == nullptr ||
              !status.ParseFromArray(data.buffer,
                                     static_cast<int>(data.buffer_size))) {
            return;
          }
          {
            std::lock_guard lock(observed.mutex);
            observed.status = std::move(status);
            ++observed.status_sequence;
          }
          observed.condition.notify_all();
        });
    tick_subscriber.SetReceiveCallback(
        [&](const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
            const eCAL::SReceiveCallbackData& data) {
          mujoco_sim::protocol::ParsedDataFrame tick;
          mujoco_sim::protocol::DataFrameError parse_error;
          if (data.buffer == nullptr ||
              !mujoco_sim::protocol::ParseDataFrame(
                  std::string_view(static_cast<const char*>(data.buffer),
                                   data.buffer_size),
                  &tick, &parse_error) ||
              tick.header.kind != mujoco_sim::protocol::DataFrameKind::kTick ||
              tick.header.session_id != accepted_session.load()) {
            return;
          }
          {
            std::lock_guard lock(observed.mutex);
            observed.tick = tick.header;
            ++observed.tick_sequence;
          }
          observed.condition.notify_all();
          if (!auto_commit.load()) return;
          auto header = tick.header;
          header.kind = mujoco_sim::protocol::DataFrameKind::kCommit;
          header.flags = 0;
          const std::array commands{0.1};
          std::string commit;
          mujoco_sim::protocol::DataFrameError encode_error;
          if (mujoco_sim::protocol::EncodeDataFrame(
                  header, commands, &commit, &encode_error)) {
            commit_publisher.Send(commit);
          }
        });
    fast_tick_subscriber.SetReceiveCallback(
        [&](const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
            const eCAL::SReceiveCallbackData& data) {
          mujoco_sim::protocol::ParsedDataFrame tick;
          mujoco_sim::protocol::DataFrameError parse_error;
          if (data.buffer == nullptr ||
              !mujoco_sim::protocol::ParseDataFrame(
                  std::string_view(static_cast<const char*>(data.buffer),
                                   data.buffer_size),
                  &tick, &parse_error) ||
              tick.header.kind != mujoco_sim::protocol::DataFrameKind::kTick ||
              tick.header.session_id != fast_session.load() ||
              !fast_auto_commit.load()) {
            return;
          }
          auto header = tick.header;
          header.kind = mujoco_sim::protocol::DataFrameKind::kCommit;
          header.flags = 0;
          const std::array commands{0.1};
          std::string commit;
          mujoco_sim::protocol::DataFrameError encode_error;
          if (!mujoco_sim::protocol::EncodeDataFrame(
                  header, commands, &commit, &encode_error)) return;
          commit_publisher.Send(commit);
          ++fast_commit_count;
        });

    const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
    while (!registry.IsConnected() &&
           std::chrono::steady_clock::now() < discovery_deadline) {
      std::this_thread::sleep_for(20ms);
    }
    std::string error;
    if (!registry.IsConnected()) {
      std::cerr << "registry discovery timed out\n";
    } else {
      auto first = Register(&registry, server_name, adapter_id, &error);
      if (first.code() != mujoco_sim::v3::OK) {
        std::cerr << "initial registration failed: " << first.message() << '\n';
      } else {
        accepted_session = first.session_id();
        std::this_thread::sleep_for(100ms);
        if (!Activate(&registry, adapter_id, first, &error)) {
          std::cerr << "initial activation failed: " << error << '\n';
        } else if (!WaitFor(&observed, [&](const auto& state) {
                     const auto* controller = FindAdapter(
                         state.status, adapter_id, first.session_id());
                     return controller != nullptr &&
                            controller->state() == mujoco_sim::v3::TIMED_OUT &&
                            controller->timeout_count() == 1;
                   })) {
          std::cerr << "initial timeout was not observed\n";
        } else if (!WaitFor(&observed, [&](const auto& state) {
                     return state.tick.session_id == first.session_id();
                   })) {
          std::cerr << "timed-out Tick was not observed\n";
        } else {
          auto fast = Register(&registry, server_name, fast_adapter_id,
                               &error, 3000, "second_pos", "second_motor");
          if (fast.code() != mujoco_sim::v3::OK) {
            std::cerr << "fast adapter registration failed: " << fast.message()
                      << '\n';
          } else {
            fast_session = fast.session_id();
            fast_auto_commit = true;
            std::this_thread::sleep_for(100ms);
            if (!Activate(&registry, fast_adapter_id, fast, &error)) {
              std::cerr << "fast adapter activation failed: " << error << '\n';
            } else {
              std::uint64_t timed_out_tick_sequence = 0;
              {
                std::lock_guard lock(observed.mutex);
                timed_out_tick_sequence = observed.tick_sequence;
              }
              if (!WaitFor(&observed, [&](const auto& state) {
                    return state.tick_sequence > timed_out_tick_sequence &&
                           state.tick.session_id == first.session_id() &&
                           fast_commit_count.load() > 0;
                  })) {
                std::cerr << "timed-out Tick was starved by fast adapter\n";
              } else {
                auto_commit = true;
                if (!WaitFor(&observed, [&](const auto& state) {
                const auto* controller = FindAdapter(
                    state.status, adapter_id, first.session_id());
                return controller != nullptr &&
                       controller->state() == mujoco_sim::v3::ACTIVE;
              })) {
                  std::cerr << "controller did not recover\n";
                } else {
                  auto_commit = false;
                  if (!WaitFor(&observed, [&](const auto& state) {
                  const auto* controller = FindAdapter(
                      state.status, adapter_id, first.session_id());
                  return controller != nullptr &&
                         controller->state() == mujoco_sim::v3::TIMED_OUT &&
                         controller->timeout_count() >= 2;
                })) {
                    std::cerr << "second timeout was not observed\n";
                  } else {
              auto replacement =
                  Register(&registry, server_name, adapter_id, &error);
              if (replacement.code() != mujoco_sim::v3::OK ||
                  replacement.session_id() == first.session_id()) {
                std::cerr << "timed-out replacement failed: "
                          << replacement.message() << '\n';
              } else {
                accepted_session = replacement.session_id();
                auto_commit = true;
                if (!Activate(&registry, adapter_id, replacement, &error)) {
                  std::cerr << "replacement activation failed: " << error
                            << '\n';
                } else if (!WaitFor(&observed, [&](const auto& state) {
                             const auto* controller = FindAdapter(
                                 state.status, adapter_id,
                                 replacement.session_id());
                             return controller != nullptr &&
                                    controller->state() ==
                                        mujoco_sim::v3::ACTIVE;
                           })) {
                  std::cerr << "replacement did not become active\n";
                } else {
                  auto_commit = false;
                  mujoco_sim::v3::UnregisterAdapterRequest request;
                  request.set_session_id(replacement.session_id());
                  request.set_epoch(replacement.epoch());
                  mujoco_sim::v3::UnregisterAdapterResponse response;
                  if (!Call(&registry,
                            mujoco_sim::protocol::kUnregisterAdapterMethod,
                            request, &response, &error) ||
                      response.code() != mujoco_sim::v3::OK) {
                    std::cerr << "runtime unregister failed: " << error << '\n';
                  } else if (!WaitFor(&observed, [&](const auto& state) {
                               return FindAdapter(
                                          state.status, adapter_id,
                                          replacement.session_id()) == nullptr &&
                                      state.status.state() ==
                                          mujoco_sim::v3::RUNNING;
                             })) {
                    std::cerr << "unregister status was not observed\n";
                  } else {
                    auto faulty = Register(&registry, server_name,
                                           adapter_id, &error, 3000);
                    if (faulty.code() != mujoco_sim::v3::OK) {
                      std::cerr << "fault injection registration failed: "
                                << faulty.message() << '\n';
                    } else {
                      accepted_session = faulty.session_id();
                      if (!Activate(&registry, adapter_id, faulty, &error) ||
                          !WaitFor(&observed, [&](const auto& state) {
                            return state.tick.session_id == faulty.session_id();
                          })) {
                        std::cerr << "fault injection activation failed\n";
                      } else {
                        mujoco_sim::protocol::DataFrameHeader fault_tick;
                        {
                          std::lock_guard lock(observed.mutex);
                          fault_tick = observed.tick;
                        }
                        fault_tick.kind =
                            mujoco_sim::protocol::DataFrameKind::kCommit;
                        fault_tick.flags = 0;
                        const std::array invalid_command{
                            std::numeric_limits<double>::quiet_NaN()};
                        std::string invalid;
                        mujoco_sim::protocol::DataFrameError encode_error;
                        if (!mujoco_sim::protocol::EncodeDataFrame(
                                fault_tick, invalid_command, &invalid,
                                &encode_error)) {
                          std::cerr << "invalid Commit encode failed\n";
                        }
                        if (invalid.empty() || !PublishUntilLocalError(
                                &commit_publisher, invalid,
                                &observed, adapter_id,
                                faulty.session_id())) {
                          std::cerr << "local ERROR status was not observed\n";
                        } else {
                          mujoco_sim::v3::UnregisterAdapterRequest cleanup;
                          cleanup.set_session_id(faulty.session_id());
                          cleanup.set_epoch(faulty.epoch());
                          mujoco_sim::v3::UnregisterAdapterResponse
                              cleanup_response;
                          if (Call(
                                  &registry,
                                  mujoco_sim::protocol::
                                      kUnregisterAdapterMethod,
                                  cleanup, &cleanup_response, &error) &&
                              cleanup_response.code() == mujoco_sim::v3::OK) {
                            mujoco_sim::v3::UnregisterAdapterRequest
                                fast_cleanup;
                            fast_cleanup.set_session_id(fast.session_id());
                            fast_cleanup.set_epoch(fast.epoch());
                            mujoco_sim::v3::UnregisterAdapterResponse
                                fast_cleanup_response;
                            if (Call(
                                    &registry,
                                    mujoco_sim::protocol::
                                        kUnregisterAdapterMethod,
                                    fast_cleanup, &fast_cleanup_response,
                                    &error) &&
                                fast_cleanup_response.code() ==
                                    mujoco_sim::v3::OK) {
                              result = 0;
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  eCAL::Finalize();
  return result;
}
