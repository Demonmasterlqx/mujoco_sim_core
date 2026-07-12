#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::adapter_sdk {

class ObserverClientTestPeer {
 public:
  static bool ValidateOptions(ObserverClient* client, std::string* error) {
    return client->ValidateOptions(error);
  }

  static void Configure(ObserverClient* client,
                        const mujoco_sim::v1::ObservationFrame& frame,
                        bool activated = true, bool registered = false) {
    client->stop_requested_ = false;
    {
      std::lock_guard lock(client->mutex_);
      auto& registration = client->registration_;
      registration.Clear();
      registration.set_code(mujoco_sim::v1::OK);
      registration.set_adapter_id(frame.observer_id());
      registration.set_role(mujoco_sim::v1::OBSERVER);
      registration.set_session_id(frame.header().session_id());
      registration.set_epoch(frame.header().epoch());
      registration.set_effective_protocol_major(
          frame.header().protocol_major());
      registration.set_effective_protocol_minor(
          frame.header().protocol_minor());
      for (const auto& block : frame.observations()) {
        auto* descriptor = registration.mutable_layout()->add_observations();
        descriptor->set_name(block.name());
        descriptor->set_type(mujoco_sim::v1::OBSERVATION_SENSOR);
        descriptor->set_dimension(block.values_size());
      }
      client->started_ = true;
      client->activated_ = activated;
      client->registered_ = registered;
      client->accepting_observations_ = activated;
      client->monitor_ecal_ = false;
      client->wait_in_progress_ = false;
      client->observations_.clear();
      client->last_sequence_ = 0;
      client->last_tick_id_ = 0;
      client->dropped_frames_ = 0;
      client->terminal_error_.clear();
      client->lifecycle_warning_.clear();
    }
    {
      std::lock_guard lock(client->callback_state_->mutex);
      client->callback_state_->client = client;
      client->callback_state_->enabled = true;
    }
  }

  static void HandleObservationBytes(
      ObserverClient* client,
      const mujoco_sim::v1::ObservationFrame& frame) {
    const std::string bytes = frame.SerializeAsString();
    client->HandleObservationBytes(bytes.data(), bytes.size());
  }

  static void HandleStatusBytes(ObserverClient* client,
                                const mujoco_sim::v1::ServerStatus& status) {
    const std::string bytes = status.SerializeAsString();
    client->HandleStatusBytes(bytes.data(), bytes.size());
  }

  static bool ActivateWith(
      ObserverClient* client,
      const std::function<bool(
          const mujoco_sim::v1::ActivateAdapterRequest&,
          mujoco_sim::v1::ActivateAdapterResponse*, std::string*)>& call,
      std::string* error) {
    std::lock_guard lifecycle_lock(client->lifecycle_mutex_);
    return client->ActivateWith(call, std::chrono::milliseconds(0), error);
  }

  static bool UnregisterWith(
      ObserverClient* client,
      const std::function<bool(
          const mujoco_sim::v1::UnregisterAdapterRequest&,
          mujoco_sim::v1::UnregisterAdapterResponse*, std::string*)>& call,
      std::string* error) {
    return client->UnregisterWith(call, error);
  }

  static mujoco_sim::v1::AdapterHeartbeat BuildHeartbeat(
      ObserverClient* client) {
    return client->BuildHeartbeat();
  }

  static void StartHeartbeatThread(ObserverClient* client) {
    client->StartHeartbeatThread();
  }

  static bool WaitInProgress(ObserverClient* client) {
    std::lock_guard lock(client->mutex_);
    return client->wait_in_progress_;
  }

  static void HoldStoppedTransitionBeforeNotify(
      ObserverClient* client, std::promise<void>* transitioned,
      std::shared_future<void> release) {
    std::unique_lock lock(client->mutex_);
    client->MarkStoppedLocked();
    transitioned->set_value();
    release.wait();
    lock.unlock();
    client->condition_.notify_all();
    client->heartbeat_condition_.notify_all();
  }

  static std::function<bool()> MakePendingCallback(ObserverClient* client) {
    const auto state = client->callback_state_;
    return [state] {
      ObserverClient::CallbackLease callback(state);
      return callback.client() != nullptr;
    };
  }

  static void HoldCallback(ObserverClient* client,
                           std::promise<void>* entered,
                           std::shared_future<void> release) {
    ObserverClient::CallbackLease callback(client->callback_state_);
    entered->set_value();
    release.wait();
  }

  static void StopObservationReception(ObserverClient* client) {
    client->StopObservationReception();
  }

  static bool WaitForObservationReceptionDisabled(
      ObserverClient* client, std::chrono::milliseconds timeout) {
    const auto state = client->callback_state_;
    std::unique_lock lock(state->mutex);
    return state->condition.wait_for(lock, timeout, [&] {
      return !state->enabled && state->client == nullptr;
    });
  }
};

}  // namespace mujoco_sim::adapter_sdk

namespace {

using namespace std::chrono_literals;
using mujoco_sim::adapter_sdk::ObservationWaitResult;
using mujoco_sim::adapter_sdk::ObserverClient;
using mujoco_sim::adapter_sdk::ObserverClientTestPeer;

template <typename T>
concept HasCommitMember = requires { &T::Commit; };

static_assert(!HasCommitMember<ObserverClient>);

mujoco_sim::adapter_sdk::ObserverOptions MakeOptions(
    std::size_t queue_capacity = 1) {
  mujoco_sim::adapter_sdk::ObserverOptions options;
  options.server_name = "default";
  options.observer_id = "watcher";
  options.observation_names = {"position", "velocity"};
  options.keepalive_period_ms = 2;
  options.queue_capacity = queue_capacity;
  return options;
}

mujoco_sim::v1::ObservationFrame MakeFrame(std::uint64_t sequence) {
  mujoco_sim::v1::ObservationFrame frame;
  frame.mutable_header()->set_server_name("default");
  frame.mutable_header()->set_session_id(41);
  frame.mutable_header()->set_epoch(7);
  frame.mutable_header()->set_tick_id(100 + sequence);
  frame.mutable_header()->set_protocol_major(
      mujoco_sim::protocol::kProtocolMajor);
  frame.mutable_header()->set_protocol_minor(
      mujoco_sim::protocol::kProtocolMinor);
  frame.set_observer_id("watcher");
  frame.set_sequence(sequence);
  auto* position = frame.add_observations();
  position->set_name("position");
  position->add_values(static_cast<double>(sequence));
  position->add_values(static_cast<double>(sequence) + 0.5);
  auto* velocity = frame.add_observations();
  velocity->set_name("velocity");
  velocity->add_values(-static_cast<double>(sequence));
  return frame;
}

mujoco_sim::v1::ServerStatus MakeStatus(std::uint64_t session_id,
                                        mujoco_sim::v1::ControllerState state,
                                        std::string message = {}) {
  mujoco_sim::v1::ServerStatus status;
  status.mutable_header()->set_server_name("default");
  status.mutable_header()->set_epoch(7);
  status.mutable_header()->set_protocol_major(
      mujoco_sim::protocol::kProtocolMajor);
  status.mutable_header()->set_protocol_minor(
      mujoco_sim::protocol::kProtocolMinor);
  status.set_state(mujoco_sim::v1::RUNNING);
  auto* adapter = status.add_adapters();
  adapter->set_adapter_id("watcher");
  adapter->set_role(mujoco_sim::v1::OBSERVER);
  adapter->set_session_id(session_id);
  adapter->set_state(state);
  adapter->set_message(std::move(message));
  return status;
}

TEST(ObserverClientTest, ValidatesDeliveryAndQueueOptions) {
  ObserverClient client(MakeOptions());
  std::string error;
  EXPECT_TRUE(ObserverClientTestPeer::ValidateOptions(&client, &error));

  auto conflicting = MakeOptions();
  conflicting.period_s = 0.1;
  ObserverClient conflicting_client(conflicting);
  EXPECT_FALSE(ObserverClientTestPeer::ValidateOptions(&conflicting_client,
                                                        &error));
  EXPECT_EQ(error,
            "exactly one of every_n_steps or period_s must be configured");

  auto periodic = MakeOptions();
  periodic.every_n_steps.reset();
  periodic.period_s = 0.01;
  ObserverClient periodic_client(periodic);
  EXPECT_TRUE(
      ObserverClientTestPeer::ValidateOptions(&periodic_client, &error));

  auto empty_queue = MakeOptions();
  empty_queue.queue_capacity = 0;
  ObserverClient empty_queue_client(empty_queue);
  EXPECT_FALSE(ObserverClientTestPeer::ValidateOptions(&empty_queue_client,
                                                        &error));
}

TEST(ObserverClientTest, DropsOldestFramesAndIgnoresStaleSequences) {
  ObserverClient client(MakeOptions(2));
  ObserverClientTestPeer::Configure(&client, MakeFrame(1));
  ObserverClientTestPeer::HandleObservationBytes(&client, MakeFrame(1));
  ObserverClientTestPeer::HandleObservationBytes(&client, MakeFrame(2));
  ObserverClientTestPeer::HandleObservationBytes(&client, MakeFrame(1));
  ObserverClientTestPeer::HandleObservationBytes(&client, MakeFrame(3));

  EXPECT_EQ(client.dropped_frames(), 1U);
  mujoco_sim::v1::ObservationFrame output;
  std::string error;
  ASSERT_EQ(client.WaitForObservation(0ms, &output, &error),
            ObservationWaitResult::kReceived);
  EXPECT_EQ(output.sequence(), 2U);
  ASSERT_EQ(client.WaitForObservation(0ms, &output, &error),
            ObservationWaitResult::kReceived);
  EXPECT_EQ(output.sequence(), 3U);
  EXPECT_EQ(client.WaitForObservation(0ms, &output, &error),
            ObservationWaitResult::kTimeout);
}

TEST(ObserverClientTest, RejectsMalformedIdentityAndObservationLayout) {
  ObserverClient client(MakeOptions());
  ObserverClientTestPeer::Configure(&client, MakeFrame(1));

  auto wrong_session = MakeFrame(1);
  wrong_session.mutable_header()->set_session_id(99);
  ObserverClientTestPeer::HandleObservationBytes(&client, wrong_session);
  auto wrong_layout = MakeFrame(2);
  wrong_layout.mutable_observations(0)->clear_values();
  ObserverClientTestPeer::HandleObservationBytes(&client, wrong_layout);

  mujoco_sim::v1::ObservationFrame output;
  output.set_observer_id("unchanged");
  std::string error;
  EXPECT_EQ(client.WaitForObservation(0ms, &output, &error),
            ObservationWaitResult::kTimeout);
  EXPECT_EQ(output.observer_id(), "unchanged");

  ObserverClientTestPeer::HandleObservationBytes(&client, MakeFrame(3));
  ASSERT_EQ(client.WaitForObservation(0ms, &output, &error),
            ObservationWaitResult::kReceived);
  EXPECT_EQ(output.sequence(), 3U);
}

TEST(ObserverClientTest, WaitTimesOutAndStopWakesWaiter) {
  ObserverClient client(MakeOptions());
  ObserverClientTestPeer::Configure(&client, MakeFrame(1));
  mujoco_sim::v1::ObservationFrame output;
  std::string error;
  EXPECT_EQ(client.WaitForObservation(1ms, &output, &error),
            ObservationWaitResult::kTimeout);
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(client.WaitForObservation(-1ms, &output, &error),
            ObservationWaitResult::kError);

  auto waiting = std::async(std::launch::async, [&] {
    std::string wait_error;
    return client.WaitForObservation(
        std::chrono::milliseconds::max(), &output, &wait_error);
  });
  std::this_thread::sleep_for(20ms);
  client.Stop();
  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), ObservationWaitResult::kStopped);
}

TEST(ObserverClientTest,
     ServerStoppedTransitionIsAtomicWithConcurrentWait) {
  ObserverClient client(MakeOptions());
  ObserverClientTestPeer::Configure(&client, MakeFrame(1));
  std::promise<void> transitioned_promise;
  auto transitioned = transitioned_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  auto stopping = std::async(std::launch::async, [&] {
    ObserverClientTestPeer::HoldStoppedTransitionBeforeNotify(
        &client, &transitioned_promise, release);
  });
  if (transitioned.wait_for(1s) != std::future_status::ready) {
    release_promise.set_value();
    stopping.wait();
    FAIL() << "stopped transition did not acquire the client mutex";
    return;
  }

  mujoco_sim::v1::ObservationFrame output;
  std::string error;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForObservation(std::chrono::milliseconds::max(),
                                     &output, &error);
  });
  EXPECT_EQ(waiting.wait_for(20ms), std::future_status::timeout);
  release_promise.set_value();
  stopping.get();
  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), ObservationWaitResult::kStopped);
  EXPECT_TRUE(error.empty());
}

TEST(ObserverClientTest, ConcurrentWaitIsRejectedAndDeliveryWakesFirstWaiter) {
  ObserverClient client(MakeOptions());
  ObserverClientTestPeer::Configure(&client, MakeFrame(1));
  std::promise<void> waiting_promise;
  auto waiting_started = waiting_promise.get_future();
  auto first_wait = std::async(std::launch::async, [&] {
    mujoco_sim::v1::ObservationFrame output;
    std::string error;
    waiting_promise.set_value();
    return client.WaitForObservation(1s, &output, &error);
  });
  ASSERT_EQ(waiting_started.wait_for(1s), std::future_status::ready);
  for (int i = 0;
       i < 100 && !ObserverClientTestPeer::WaitInProgress(&client); ++i) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(ObserverClientTestPeer::WaitInProgress(&client));

  mujoco_sim::v1::ObservationFrame output;
  std::string error;
  EXPECT_EQ(client.WaitForObservation(0ms, &output, &error),
            ObservationWaitResult::kError);
  EXPECT_EQ(error,
            "another thread is already waiting for an observation");
  ObserverClientTestPeer::HandleObservationBytes(&client, MakeFrame(1));
  EXPECT_EQ(first_wait.get(), ObservationWaitResult::kReceived);
}

TEST(ObserverClientTest, ServerErrorAndReplacementAreTerminal) {
  ObserverClient error_client(MakeOptions());
  ObserverClientTestPeer::Configure(&error_client, MakeFrame(1));
  ObserverClientTestPeer::HandleStatusBytes(
      &error_client, MakeStatus(41, mujoco_sim::v1::ERROR, "sensor failed"));
  mujoco_sim::v1::ObservationFrame output;
  std::string error;
  EXPECT_EQ(error_client.WaitForObservation(0ms, &output, &error),
            ObservationWaitResult::kError);
  EXPECT_EQ(error, "sensor failed");
  EXPECT_EQ(error_client.lifecycle_warning(), "sensor failed");

  ObserverClient replacement_client(MakeOptions());
  ObserverClientTestPeer::Configure(&replacement_client, MakeFrame(1));
  ObserverClientTestPeer::HandleStatusBytes(
      &replacement_client,
      MakeStatus(42, mujoco_sim::v1::ACTIVE));
  EXPECT_EQ(replacement_client.WaitForObservation(0ms, &output, &error),
            ObservationWaitResult::kError);
  EXPECT_EQ(error, "server replaced observer session");
}

TEST(ObserverClientTest, ServerTerminalStatesWakeInfiniteWait) {
  ObserverClient stopped_client(MakeOptions());
  ObserverClientTestPeer::Configure(&stopped_client, MakeFrame(1));
  mujoco_sim::v1::ObservationFrame stopped_output;
  std::string stopped_error;
  auto stopped_wait = std::async(std::launch::async, [&] {
    return stopped_client.WaitForObservation(
        std::chrono::milliseconds::max(), &stopped_output, &stopped_error);
  });
  std::this_thread::sleep_for(20ms);
  auto stopped_status = MakeStatus(41, mujoco_sim::v1::ACTIVE);
  stopped_status.set_state(mujoco_sim::v1::STOPPED);
  ObserverClientTestPeer::HandleStatusBytes(&stopped_client, stopped_status);
  ASSERT_EQ(stopped_wait.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(stopped_wait.get(), ObservationWaitResult::kStopped);
  EXPECT_TRUE(stopped_error.empty());

  ObserverClient stopped_before_wait(MakeOptions());
  ObserverClientTestPeer::Configure(&stopped_before_wait, MakeFrame(1));
  ObserverClientTestPeer::HandleStatusBytes(&stopped_before_wait,
                                             stopped_status);
  EXPECT_EQ(stopped_before_wait.WaitForObservation(
                0ms, &stopped_output, &stopped_error),
            ObservationWaitResult::kStopped);
  EXPECT_TRUE(stopped_error.empty());

  ObserverClient error_client(MakeOptions());
  ObserverClientTestPeer::Configure(&error_client, MakeFrame(1));
  mujoco_sim::v1::ObservationFrame error_output;
  std::string error_message;
  auto error_wait = std::async(std::launch::async, [&] {
    return error_client.WaitForObservation(
        std::chrono::milliseconds::max(), &error_output, &error_message);
  });
  std::this_thread::sleep_for(20ms);
  auto error_status = MakeStatus(41, mujoco_sim::v1::ACTIVE);
  error_status.set_state(mujoco_sim::v1::ERROR_PAUSED);
  error_status.set_message("fatal simulation error");
  ObserverClientTestPeer::HandleStatusBytes(&error_client, error_status);
  ASSERT_EQ(error_wait.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(error_wait.get(), ObservationWaitResult::kError);
  EXPECT_EQ(error_message, "fatal simulation error");
}

TEST(ObserverClientTest, StatusRequiresNegotiatedProtocolMinor) {
  auto frame = MakeFrame(1);
  const std::uint32_t negotiated_minor =
      mujoco_sim::protocol::kProtocolMinor == 0
          ? 1
          : mujoco_sim::protocol::kProtocolMinor - 1;
  frame.mutable_header()->set_protocol_minor(negotiated_minor);
  ObserverClient client(MakeOptions());
  ObserverClientTestPeer::Configure(&client, frame);

  auto mismatched =
      MakeStatus(41, mujoco_sim::v1::ERROR, "wrong protocol minor");
  mismatched.mutable_header()->set_protocol_minor(negotiated_minor + 1);
  ObserverClientTestPeer::HandleStatusBytes(&client, mismatched);

  mujoco_sim::v1::ObservationFrame output;
  std::string error;
  EXPECT_EQ(client.WaitForObservation(0ms, &output, &error),
            ObservationWaitResult::kTimeout);
  EXPECT_TRUE(client.lifecycle_warning().empty());

  auto matching = MakeStatus(41, mujoco_sim::v1::ERROR, "matching error");
  matching.mutable_header()->set_protocol_minor(negotiated_minor);
  ObserverClientTestPeer::HandleStatusBytes(&client, matching);
  EXPECT_EQ(client.WaitForObservation(0ms, &output, &error),
            ObservationWaitResult::kError);
  EXPECT_EQ(error, "matching error");
}

TEST(ObserverClientTest, ActivateAndUnregisterUseObserverRole) {
  ObserverClient client(MakeOptions());
  ObserverClientTestPeer::Configure(&client, MakeFrame(1), false, true);
  std::string error;
  ASSERT_TRUE(ObserverClientTestPeer::ActivateWith(
      &client,
      [](const auto& request, auto* response, std::string*) {
        EXPECT_EQ(request.adapter_id(), "watcher");
        EXPECT_EQ(request.role(), mujoco_sim::v1::OBSERVER);
        EXPECT_EQ(request.session_id(), 41U);
        response->set_code(mujoco_sim::v1::OK);
        response->set_state(mujoco_sim::v1::ACTIVE);
        return true;
      },
      &error))
      << error;
  EXPECT_EQ(client.registration().role(), mujoco_sim::v1::OBSERVER);

  EXPECT_TRUE(ObserverClientTestPeer::UnregisterWith(
      &client,
      [](const auto& request, auto* response, std::string*) {
        EXPECT_EQ(request.adapter_id(), "watcher");
        EXPECT_EQ(request.role(), mujoco_sim::v1::OBSERVER);
        response->set_code(mujoco_sim::v1::NOT_FOUND);
        return true;
      },
      &error));
}

TEST(ObserverClientTest, SuccessfulUnregisterWakesConcurrentWaitAsStopped) {
  ObserverClient client(MakeOptions());
  ObserverClientTestPeer::Configure(&client, MakeFrame(1), true, true);
  mujoco_sim::v1::ObservationFrame output;
  std::string wait_error;
  auto waiting = std::async(std::launch::async, [&] {
    return client.WaitForObservation(std::chrono::milliseconds::max(),
                                     &output, &wait_error);
  });
  for (int i = 0;
       i < 100 && !ObserverClientTestPeer::WaitInProgress(&client); ++i) {
    std::this_thread::sleep_for(1ms);
  }
  if (!ObserverClientTestPeer::WaitInProgress(&client)) {
    client.Stop();
    waiting.wait();
    FAIL() << "wait did not enter the blocking state";
    return;
  }

  std::string unregister_error;
  const bool unregistered = ObserverClientTestPeer::UnregisterWith(
      &client,
      [](const auto&, auto* response, std::string*) {
        response->set_code(mujoco_sim::v1::OK);
        return true;
      },
      &unregister_error);
  if (!unregistered) {
    client.Stop();
    waiting.wait();
    FAIL() << unregister_error;
    return;
  }
  ASSERT_EQ(waiting.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(waiting.get(), ObservationWaitResult::kStopped);
  EXPECT_TRUE(wait_error.empty());
}

TEST(ObserverClientTest, StopWinsRaceWithSuccessfulActivationResponse) {
  ObserverClient client(MakeOptions());
  ObserverClientTestPeer::Configure(&client, MakeFrame(1), false, true);
  std::promise<void> entered_promise;
  auto entered = entered_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  std::string error;
  auto activating = std::async(std::launch::async, [&] {
    return ObserverClientTestPeer::ActivateWith(
        &client,
        [&](const auto&, auto* response, std::string*) {
          entered_promise.set_value();
          release.wait();
          response->set_code(mujoco_sim::v1::OK);
          response->set_state(mujoco_sim::v1::ACTIVE);
          return true;
        },
        &error);
  });
  ASSERT_EQ(entered.wait_for(1s), std::future_status::ready);
  client.Stop();
  release_promise.set_value();
  EXPECT_FALSE(activating.get());
  EXPECT_EQ(error, "observer activation was cancelled");
}

TEST(ObserverClientTest, HeartbeatTracksLatestAcceptedObservation) {
  ObserverClient client(MakeOptions());
  ObserverClientTestPeer::Configure(&client, MakeFrame(1), true, true);
  ObserverClientTestPeer::HandleObservationBytes(&client, MakeFrame(5));
  const auto heartbeat = ObserverClientTestPeer::BuildHeartbeat(&client);
  EXPECT_EQ(heartbeat.adapter_id(), "watcher");
  EXPECT_EQ(heartbeat.role(), mujoco_sim::v1::OBSERVER);
  EXPECT_EQ(heartbeat.header().session_id(), 41U);
  EXPECT_EQ(heartbeat.header().tick_id(), 105U);
  EXPECT_EQ(heartbeat.last_observation_sequence(), 5U);

  ObserverClientTestPeer::StartHeartbeatThread(&client);
  std::this_thread::sleep_for(5ms);
  client.Stop();
}

TEST(ObserverClientTest, TeardownDrainsAndDisablesCallbacks) {
  ObserverClient client(MakeOptions());
  ObserverClientTestPeer::Configure(&client, MakeFrame(1));
  auto delayed = ObserverClientTestPeer::MakePendingCallback(&client);
  EXPECT_TRUE(delayed());

  std::promise<void> entered_promise;
  auto entered = entered_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  auto callback = std::async(std::launch::async, [&] {
    ObserverClientTestPeer::HoldCallback(&client, &entered_promise, release);
  });
  ASSERT_EQ(entered.wait_for(1s), std::future_status::ready);
  auto stopping = std::async(std::launch::async, [&] {
    ObserverClientTestPeer::StopObservationReception(&client);
  });
  EXPECT_TRUE(ObserverClientTestPeer::WaitForObservationReceptionDisabled(
      &client, 1s));
  EXPECT_EQ(stopping.wait_for(0ms), std::future_status::timeout);
  release_promise.set_value();
  callback.get();
  ASSERT_EQ(stopping.wait_for(1s), std::future_status::ready);
  stopping.get();
  EXPECT_FALSE(delayed());
}

TEST(ObserverClientTest, DelayedCallbackCannotReachDestroyedClient) {
  std::function<bool()> delayed;
  {
    auto client = std::make_unique<ObserverClient>(MakeOptions());
    ObserverClientTestPeer::Configure(client.get(), MakeFrame(1));
    delayed = ObserverClientTestPeer::MakePendingCallback(client.get());
    EXPECT_TRUE(delayed());
  }
  EXPECT_FALSE(delayed());
}

}  // namespace
