#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>

#include "../src/internal/controller_client_test_peer.hpp"

namespace {

using namespace std::chrono_literals;
using mujoco_sim::adapter_sdk::ControllerClient;
using mujoco_sim::adapter_sdk::ControllerClientTestPeer;
using mujoco_sim::adapter_sdk::ControllerOptions;
using mujoco_sim::adapter_sdk::FaultCode;
using mujoco_sim::adapter_sdk::StateLease;
using mujoco_sim::adapter_sdk::StateReadResult;
using mujoco_sim::adapter_sdk::internal::PublishResult;

class FakeControllerTransport final : public mujoco_sim::adapter_sdk::internal::AdapterTransport {
 public:
  bool Register(const mujoco_sim::v4::RegisterAdapterRequest&,
                mujoco_sim::v4::RegisterAdapterResponse*, std::string*) override {
    return false;
  }
  bool Activate(const mujoco_sim::v4::ActivateAdapterRequest&,
                mujoco_sim::v4::ActivateAdapterResponse* response, std::string*) override {
    std::unique_lock lock(mutex_);
    activate_entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [&] { return !block_activate_; });
    response->set_code(mujoco_sim::v4::OK);
    response->set_state(mujoco_sim::v4::ACTIVE_NO_COMMAND);
    return true;
  }
  bool Unregister(const mujoco_sim::v4::UnregisterAdapterRequest& request,
                  mujoco_sim::v4::UnregisterAdapterResponse* response, std::string*) override {
    std::lock_guard lock(mutex_);
    ++unregister_calls_;
    unregister_requests_.push_back(request);
    response->set_code(unregister_code_);
    return unregister_calls_ > failed_unregister_calls_;
  }
  PublishResult PublishCommand(std::string_view command) override {
    std::lock_guard lock(mutex_);
    commands_.emplace_back(command);
    const std::size_t attempt = commands_.size() - 1;
    condition_.notify_all();
    if (throw_on_publish_) throw std::runtime_error("injected command publish failure");
    if (!publish_error_.empty()) return {false, publish_error_};
    return {attempt >= outcomes_.size() || outcomes_[attempt], {}};
  }
  PublishResult PublishFault(const mujoco_sim::v4::ControllerFault&) override {
    std::lock_guard lock(mutex_);
    return fault_result_;
  }
  PublishResult PublishHeartbeat(const mujoco_sim::v4::SessionHeartbeat&) override {
    return {true, {}};
  }

  void SetOutcomes(std::vector<bool> outcomes) {
    std::lock_guard lock(mutex_);
    outcomes_ = std::move(outcomes);
  }

  void ThrowOnPublish() {
    std::lock_guard lock(mutex_);
    throw_on_publish_ = true;
  }

  void FailPublish(std::string error) {
    std::lock_guard lock(mutex_);
    publish_error_ = std::move(error);
  }

  void SetFaultResult(PublishResult result) {
    std::lock_guard lock(mutex_);
    fault_result_ = std::move(result);
  }

  bool WaitForCommands(std::size_t count, std::chrono::milliseconds timeout) {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [&] { return commands_.size() >= count; });
  }

  void BlockActivate() {
    std::lock_guard lock(mutex_);
    block_activate_ = true;
    activate_entered_ = false;
  }

  bool WaitForActivate(std::chrono::milliseconds timeout) {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [&] { return activate_entered_; });
  }

  void ReleaseActivate() {
    {
      std::lock_guard lock(mutex_);
      block_activate_ = false;
    }
    condition_.notify_all();
  }

  std::vector<std::string> Commands() const {
    std::lock_guard lock(mutex_);
    return commands_;
  }

  void FailUnregisterCalls(std::size_t count) {
    std::lock_guard lock(mutex_);
    failed_unregister_calls_ = count;
  }

  void SetUnregisterCode(mujoco_sim::v4::ErrorCode code) {
    std::lock_guard lock(mutex_);
    unregister_code_ = code;
  }

  std::size_t UnregisterCalls() const {
    std::lock_guard lock(mutex_);
    return unregister_calls_;
  }

  std::vector<mujoco_sim::v4::UnregisterAdapterRequest> UnregisterRequests() const {
    std::lock_guard lock(mutex_);
    return unregister_requests_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<bool> outcomes_;
  std::vector<std::string> commands_;
  std::size_t failed_unregister_calls_ = 0;
  std::size_t unregister_calls_ = 0;
  std::vector<mujoco_sim::v4::UnregisterAdapterRequest> unregister_requests_;
  mujoco_sim::v4::ErrorCode unregister_code_ = mujoco_sim::v4::OK;
  bool block_activate_ = false;
  bool activate_entered_ = false;
  bool throw_on_publish_ = false;
  std::string publish_error_;
  PublishResult fault_result_{true, {}};
};

ControllerOptions Options() {
  ControllerOptions options;
  options.adapter_id = "controller";
  options.observation_names = {"position", "velocity"};
  options.command_names = {"actuator"};
  return options;
}

bool WaitUntilReading(ControllerClient* client) {
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (ControllerClientTestPeer::IsWaiting(client)) return true;
    std::this_thread::yield();
  }
  return false;
}

std::pair<mujoco_sim::protocol::DataFrameHeader, std::vector<double>> Decode(
    const std::string& bytes) {
  mujoco_sim::protocol::ParsedDataFrame parsed;
  mujoco_sim::protocol::DataFrameError error;
  EXPECT_TRUE(mujoco_sim::protocol::ParseDataFrame(bytes, &parsed, &error)) << error.message;
  std::vector<double> scalars(parsed.scalar_count);
  EXPECT_TRUE(mujoco_sim::protocol::DecodeDataFrameScalars(parsed, scalars, &error))
      << error.message;
  return {parsed.header, std::move(scalars)};
}

TEST(ControllerClientTest, StateLeaseIsMoveOnlyAndRetainsImmutableFrame) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  ControllerClientTestPeer::Configure(&client, transport);
  const std::array observations{1.5, -2.0};
  ControllerClientTestPeer::HandleState(&client, 3, 17, observations);

  StateLease lease;
  std::string error;
  ASSERT_EQ(client.ReadLatestState(&lease, &error), StateReadResult::kReceived) << error;
  StateLease moved(std::move(lease));
  EXPECT_FALSE(lease);
  ASSERT_TRUE(moved);
  EXPECT_EQ(moved.sequence(), 3U);
  EXPECT_EQ(moved.tick_id(), 17U);
  EXPECT_DOUBLE_EQ(moved.simulation_time_s(), 0.25);
  ASSERT_EQ(moved.observations().size(), 2U);
  EXPECT_DOUBLE_EQ(moved.observations()[0], 1.5);

  StateLease latest;
  ASSERT_EQ(client.ReadLatestState(&latest, &error), StateReadResult::kReceived);
  EXPECT_EQ(latest.sequence(), moved.sequence());
}

TEST(ControllerClientTest, WaitRequiresNewStateAndRejectsSecondWaiter) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  ControllerClientTestPeer::Configure(&client, transport);
  const std::array first_observations{1.0, 2.0};
  ControllerClientTestPeer::HandleState(&client, 1, 10, first_observations);
  StateLease first;
  std::string error;
  ASSERT_EQ(client.ReadLatestState(&first, &error), StateReadResult::kReceived);

  auto waiting = std::async(std::launch::async, [&] {
    StateLease frame;
    std::string wait_error;
    return std::pair{client.WaitForNewState(1s, &frame, &wait_error), frame.sequence()};
  });
  ASSERT_TRUE(WaitUntilReading(&client));

  StateLease competing;
  EXPECT_EQ(client.WaitForNewState(0ms, &competing, &error), StateReadResult::kError);
  EXPECT_EQ(error, "another state read is already in progress");

  const std::array next_observations{3.0, 4.0};
  ControllerClientTestPeer::HandleState(&client, 2, 11, next_observations);
  const auto result = waiting.get();
  EXPECT_EQ(result.first, StateReadResult::kReceived);
  EXPECT_EQ(result.second, 2U);
}

TEST(ControllerClientTest, StopWakesBlockingStateWait) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  ControllerClientTestPeer::Configure(&client, transport);
  auto waiting = std::async(std::launch::async, [&] {
    StateLease frame;
    std::string error;
    return client.WaitForNewState(std::chrono::milliseconds::max(), &frame, &error);
  });
  ASSERT_TRUE(WaitUntilReading(&client));
  client.Stop();
  EXPECT_EQ(waiting.get(), StateReadResult::kStopped);
}

TEST(ControllerClientTest, StopCancelsActivationAtRpcCommitBoundary) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  ControllerClientTestPeer::Configure(&client, transport);
  ControllerClientTestPeer::PrepareActivation(&client);
  ControllerClientTestPeer::EnableRemoteRegistration(&client);
  transport->BlockActivate();

  std::string activation_error;
  auto activating =
      std::async(std::launch::async, [&] { return client.Activate(&activation_error); });
  ASSERT_TRUE(transport->WaitForActivate(1s));

  auto stopping = std::async(std::launch::async, [&] { client.Stop(); });
  const auto stop_deadline = std::chrono::steady_clock::now() + 1s;
  while (!ControllerClientTestPeer::StopRequested(&client) &&
         std::chrono::steady_clock::now() < stop_deadline) {
    std::this_thread::yield();
  }
  ASSERT_TRUE(ControllerClientTestPeer::StopRequested(&client));
  transport->ReleaseActivate();

  EXPECT_FALSE(activating.get());
  EXPECT_EQ(activation_error, "controller activation was cancelled");
  EXPECT_EQ(transport->UnregisterCalls(), 1U);
  ASSERT_EQ(stopping.wait_for(1s), std::future_status::ready);
  stopping.get();
}

TEST(ControllerClientTest, PublisherStartFailureCompensatesRemoteActivation) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  ControllerClientTestPeer::Configure(&client, transport);
  ControllerClientTestPeer::PrepareActivation(&client);
  ControllerClientTestPeer::EnableRemoteRegistration(&client);
  ControllerClientTestPeer::FailPublisherStart(&client);

  std::string error;
  EXPECT_FALSE(client.Activate(&error));
  EXPECT_NE(error.find("failed to start Controller publisher"), std::string::npos) << error;
  EXPECT_NE(error.find("injected publisher thread creation failure"), std::string::npos) << error;
  EXPECT_EQ(transport->UnregisterCalls(), 1U);
  const auto requests = transport->UnregisterRequests();
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().session_id(), 42U);
  EXPECT_EQ(requests.front().epoch(), 7U);
  EXPECT_TRUE(client.Unregister(&error)) << error;
  EXPECT_EQ(transport->UnregisterCalls(), 1U);
}

TEST(ControllerClientTest, LatestCommandWinsBeforePublish) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  ControllerClientTestPeer::Configure(&client, transport, 1s);
  std::string error;
  const std::array first{1.0, 2.0};
  const std::array latest{3.0, 4.0};
  ASSERT_TRUE(client.SetCommand(first, &error)) << error;
  ASSERT_TRUE(client.SetCommand(latest, &error)) << error;
  ControllerClientTestPeer::StartPublisher(&client);
  ASSERT_TRUE(transport->WaitForCommands(1, 1s));
  client.Stop();

  const auto commands = transport->Commands();
  ASSERT_EQ(commands.size(), 1U);
  const auto [header, scalars] = Decode(commands.front());
  EXPECT_EQ(header.kind, mujoco_sim::protocol::DataFrameKind::kControllerCommand);
  EXPECT_EQ(header.sequence, 1U);
  EXPECT_EQ(scalars, std::vector<double>({3.0, 4.0}));
}

TEST(ControllerClientTest, FailedSendRetriesIdenticalBytesThenKeepaliveAdvances) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  transport->SetOutcomes({false, false, true, true});
  ControllerClientTestPeer::Configure(&client, transport, 20ms);
  std::string error;
  const std::array command{5.0, -6.0};
  ASSERT_TRUE(client.SetCommand(command, &error)) << error;
  ControllerClientTestPeer::StartPublisher(&client);
  ASSERT_TRUE(transport->WaitForCommands(4, 2s));
  client.Stop();

  const auto commands = transport->Commands();
  ASSERT_GE(commands.size(), 4U);
  EXPECT_EQ(commands[0], commands[1]);
  EXPECT_EQ(commands[1], commands[2]);
  const auto [retry_header, retry_scalars] = Decode(commands[2]);
  const auto [keepalive_header, keepalive_scalars] = Decode(commands[3]);
  EXPECT_EQ(retry_header.sequence, 1U);
  EXPECT_EQ(keepalive_header.sequence, 2U);
  EXPECT_EQ(retry_scalars, keepalive_scalars);
  EXPECT_EQ(retry_scalars, std::vector<double>({5.0, -6.0}));
}

TEST(ControllerClientTest, PublisherExceptionBecomesTerminalReadError) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  transport->ThrowOnPublish();
  ControllerClientTestPeer::Configure(&client, transport);
  std::string error;
  const std::array command{5.0, -6.0};
  ASSERT_TRUE(client.SetCommand(command, &error)) << error;
  ControllerClientTestPeer::StartPublisher(&client);
  ASSERT_TRUE(transport->WaitForCommands(1, 1s));

  StateLease state;
  EXPECT_EQ(client.WaitForNewState(1s, &state, &error), StateReadResult::kError);
  EXPECT_NE(error.find("Controller publisher thread failed"), std::string::npos) << error;
  EXPECT_NE(error.find("injected command publish failure"), std::string::npos) << error;
}

TEST(ControllerClientTest, PublisherTransportErrorBecomesTerminalReadError) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  transport->FailPublish("eCAL send failed: injected diagnostic");
  ControllerClientTestPeer::Configure(&client, transport);
  std::string error;
  const std::array command{5.0, -6.0};
  ASSERT_TRUE(client.SetCommand(command, &error)) << error;
  ControllerClientTestPeer::StartPublisher(&client);
  ASSERT_TRUE(transport->WaitForCommands(1, 1s));

  StateLease state;
  EXPECT_EQ(client.WaitForNewState(1s, &state, &error), StateReadResult::kError);
  EXPECT_NE(error.find("failed to publish ControllerCommand"), std::string::npos) << error;
  EXPECT_NE(error.find("injected diagnostic"), std::string::npos) << error;
}

TEST(ControllerClientTest, FaultPublicationPreservesTransportDiagnostic) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  transport->SetFaultResult({false, "injected fault serialization failure"});
  ControllerClientTestPeer::Configure(&client, transport);
  ControllerClientTestPeer::PrepareActivation(&client);
  std::string error;
  ASSERT_TRUE(client.Activate(&error)) << error;

  EXPECT_FALSE(client.ReportFault(FaultCode::kControllerError, "failed", &error));
  EXPECT_NE(error.find("failed to publish ControllerFault"), std::string::npos) << error;
  EXPECT_NE(error.find("injected fault serialization failure"), std::string::npos) << error;
}

TEST(ControllerClientTest, FailedUnregisterPreservesSessionForRetry) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  transport->FailUnregisterCalls(1);
  ControllerClientTestPeer::Configure(&client, transport);
  ControllerClientTestPeer::EnableRemoteRegistration(&client);

  std::string error;
  EXPECT_FALSE(client.Unregister(&error));
  EXPECT_TRUE(client.Unregister(&error)) << error;
  EXPECT_EQ(transport->UnregisterCalls(), 2U);
  const auto requests = transport->UnregisterRequests();
  ASSERT_EQ(requests.size(), 2U);
  EXPECT_EQ(requests[0].session_id(), 42U);
  EXPECT_EQ(requests[0].epoch(), 7U);
  EXPECT_EQ(requests[1].SerializeAsString(), requests[0].SerializeAsString());
}

TEST(ControllerClientTest, MissingRemoteSessionCompletesIdempotentUnregister) {
  ControllerClient client(Options());
  auto transport = std::make_shared<FakeControllerTransport>();
  transport->SetUnregisterCode(mujoco_sim::v4::INVALID_SESSION);
  ControllerClientTestPeer::Configure(&client, transport);
  ControllerClientTestPeer::EnableRemoteRegistration(&client);

  std::string error;
  EXPECT_TRUE(client.Unregister(&error)) << error;
  EXPECT_EQ(transport->UnregisterCalls(), 1U);
  EXPECT_TRUE(client.Unregister(&error)) << error;
  EXPECT_EQ(transport->UnregisterCalls(), 1U);
}

static_assert(!std::is_copy_constructible_v<StateLease>);
static_assert(!std::is_copy_assignable_v<StateLease>);
static_assert(std::is_nothrow_move_constructible_v<StateLease>);
static_assert(std::is_nothrow_move_assignable_v<StateLease>);

}  // namespace
