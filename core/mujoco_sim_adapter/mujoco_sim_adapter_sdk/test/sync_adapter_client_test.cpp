#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>

#include "../src/internal/sync_adapter_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {

class FakeAdapterTransport final : public internal::AdapterTransport {
 public:
  bool Register(const mujoco_sim::v3::RegisterAdapterRequest&,
                mujoco_sim::v3::RegisterAdapterResponse*, std::string*) override {
    return false;
  }
  bool Activate(const mujoco_sim::v3::ActivateAdapterRequest&,
                mujoco_sim::v3::ActivateAdapterResponse* response,
                std::string*) override {
    response->set_code(mujoco_sim::v3::OK);
    return true;
  }
  bool Unregister(const mujoco_sim::v3::UnregisterAdapterRequest&,
                  mujoco_sim::v3::UnregisterAdapterResponse* response,
                  std::string*) override {
    response->set_code(mujoco_sim::v3::OK);
    return true;
  }
  bool PublishCommit(std::string_view bytes) override {
    commits.emplace_back(bytes);
    return commit_ok;
  }
  bool PublishFault(const mujoco_sim::v3::ControllerFault& value) override {
    fault = value;
    return fault_ok;
  }
  void PublishHeartbeat(const mujoco_sim::v3::SessionHeartbeat& value) override {
    heartbeat = value;
  }

  bool commit_ok = true;
  bool fault_ok = true;
  std::vector<std::string> commits;
  mujoco_sim::v3::ControllerFault fault;
  mujoco_sim::v3::SessionHeartbeat heartbeat;
};

class SyncAdapterClientTestPeer {
 public:
  static bool ValidateOptions(SyncAdapterClient* client, std::string* error) {
    return client->impl_->ValidateOptions(error);
  }

  static std::shared_ptr<FakeAdapterTransport> Configure(
      SyncAdapterClient* client) {
    auto transport = std::make_shared<FakeAdapterTransport>();
    client->impl_->stop_requested_ = false;
    std::lock_guard lock(client->impl_->mutex_);
    client->impl_->registration_.adapter_id = "arm";
    client->impl_->registration_.session_id = 42;
    client->impl_->registration_.epoch = 7;
    client->impl_->registration_.layout = ControllerLayout(
        {{"position", {0, 2}, false, 0.0, 0.0}},
        {{"motor", {0, 2}, true, -1.0, 1.0}}, 2, 2);
    client->impl_->layout_fingerprint_ = {11, 12};
    client->impl_->session_ = internal::AdapterSession{};
    client->impl_->session_.Registered({42, 7});
    client->impl_->session_.StartCompleted();
    client->impl_->session_.BeginActivation();
    client->impl_->session_.Activated();
    client->impl_->monitor_ecal_ = false;
    client->impl_->transport_ = transport;
    client->impl_->queued_tick_.reset();
    client->impl_->outstanding_tick_.reset();
    client->impl_->committed_tick_.reset();
    client->impl_->cached_commit_.reset();
    client->impl_->terminal_error_.clear();
    return transport;
  }

  static void HandleTick(SyncAdapterClient* client, std::uint64_t tick_id,
                         std::array<double, 2> scalars = {1.0, 2.0}) {
    mujoco_sim::protocol::DataFrameHeader header;
    header.kind = mujoco_sim::protocol::DataFrameKind::kTick;
    header.session_id = 42;
    header.epoch = 7;
    header.tick_id = tick_id;
    header.layout = {11, 12};
    header.simulation_time_s = 0.25;
    std::string bytes;
    mujoco_sim::protocol::DataFrameError error;
    ASSERT_TRUE(mujoco_sim::protocol::EncodeDataFrame(
        header, scalars, &bytes, &error));
    client->impl_->HandleTickBytes(bytes.data(), bytes.size());
  }
};

}  // namespace mujoco_sim::adapter_sdk

namespace {

using namespace std::chrono_literals;
using mujoco_sim::adapter_sdk::AdapterOptions;
using mujoco_sim::adapter_sdk::FaultCode;
using mujoco_sim::adapter_sdk::SyncAdapterClient;
using mujoco_sim::adapter_sdk::SyncAdapterClientTestPeer;
using mujoco_sim::adapter_sdk::TickLease;
using mujoco_sim::adapter_sdk::TickWaitResult;

AdapterOptions Options() {
  AdapterOptions options;
  options.adapter_id = "arm";
  options.observation_names = {"position"};
  options.command_names = {"motor"};
  return options;
}

TEST(SyncAdapterClientTest, ValidatesHeartbeatRatio) {
  std::string error;
  SyncAdapterClient valid(Options());
  EXPECT_TRUE(SyncAdapterClientTestPeer::ValidateOptions(&valid, &error));
  auto options = Options();
  options.heartbeat_period_ms = 400;
  SyncAdapterClient invalid(options);
  EXPECT_FALSE(SyncAdapterClientTestPeer::ValidateOptions(&invalid, &error));
}

TEST(SyncAdapterClientTest, DecodesDenseTickAndExposesLayoutSpans) {
  SyncAdapterClient client(Options());
  SyncAdapterClientTestPeer::Configure(&client);
  SyncAdapterClientTestPeer::HandleTick(&client, 5);
  TickLease tick;
  std::string error;
  ASSERT_EQ(client.WaitForTick(0ms, &tick, &error), TickWaitResult::kReceived);
  EXPECT_EQ(tick.tick_id(), 5U);
  EXPECT_DOUBLE_EQ(tick.simulation_time_s(), 0.25);
  ASSERT_EQ(tick.observations().size(), 2U);
  EXPECT_DOUBLE_EQ(tick.observations()[1], 2.0);
  ASSERT_TRUE(client.layout().Observation("position"));
  EXPECT_EQ(client.layout().Observation("position")->count, 2U);
}

TEST(SyncAdapterClientTest, CommitUsesDenseBytesAndDuplicateIsExactReplay) {
  SyncAdapterClient client(Options());
  const auto transport = SyncAdapterClientTestPeer::Configure(&client);
  SyncAdapterClientTestPeer::HandleTick(&client, 5);
  TickLease tick;
  std::string error;
  ASSERT_EQ(client.WaitForTick(0ms, &tick, &error), TickWaitResult::kReceived);
  const std::array commands{0.2, -0.3};
  ASSERT_TRUE(client.Commit(tick, commands, &error)) << error;
  ASSERT_TRUE(client.Commit(tick, commands, &error)) << error;
  ASSERT_EQ(transport->commits.size(), 2U);
  EXPECT_EQ(transport->commits[0], transport->commits[1]);
  mujoco_sim::protocol::ParsedDataFrame parsed;
  mujoco_sim::protocol::DataFrameError parse_error;
  ASSERT_TRUE(mujoco_sim::protocol::ParseDataFrame(
      transport->commits[0], &parsed, &parse_error));
  EXPECT_EQ(parsed.header.kind,
            mujoco_sim::protocol::DataFrameKind::kCommit);
  EXPECT_EQ(parsed.scalar_count, 2U);
}

TEST(SyncAdapterClientTest, RejectsInvalidCommandCountRangeAndBits) {
  SyncAdapterClient client(Options());
  SyncAdapterClientTestPeer::Configure(&client);
  SyncAdapterClientTestPeer::HandleTick(&client, 1);
  TickLease tick;
  std::string error;
  ASSERT_EQ(client.WaitForTick(0ms, &tick, &error), TickWaitResult::kReceived);
  const std::array too_few{0.0};
  EXPECT_FALSE(client.Commit(tick, too_few, &error));
  const std::array out_of_range{2.0, 0.0};
  EXPECT_FALSE(client.Commit(tick, out_of_range, &error));
}

TEST(SyncAdapterClientTest, DuplicateTickRetriesCachedCommitWithoutHeartbeat) {
  SyncAdapterClient client(Options());
  const auto transport = SyncAdapterClientTestPeer::Configure(&client);
  SyncAdapterClientTestPeer::HandleTick(&client, 2);
  TickLease tick;
  std::string error;
  ASSERT_EQ(client.WaitForTick(0ms, &tick, &error), TickWaitResult::kReceived);
  const std::array commands{0.1, 0.2};
  ASSERT_TRUE(client.Commit(tick, commands, &error));
  SyncAdapterClientTestPeer::HandleTick(&client, 2);
  ASSERT_EQ(transport->commits.size(), 2U);
  EXPECT_EQ(transport->commits[0], transport->commits[1]);
  EXPECT_EQ(transport->heartbeat.session_id(), 0U);
}

TEST(SyncAdapterClientTest, ConflictingDuplicateTerminatesSession) {
  SyncAdapterClient client(Options());
  SyncAdapterClientTestPeer::Configure(&client);
  SyncAdapterClientTestPeer::HandleTick(&client, 3, {1.0, 2.0});
  SyncAdapterClientTestPeer::HandleTick(&client, 3, {1.0, 4.0});
  TickLease tick;
  std::string error;
  EXPECT_EQ(client.WaitForTick(0ms, &tick, &error), TickWaitResult::kError);
  EXPECT_NE(error.find("conflicting payload"), std::string::npos);
}

TEST(SyncAdapterClientTest, FailTickPublishesControllerFault) {
  SyncAdapterClient client(Options());
  const auto transport = SyncAdapterClientTestPeer::Configure(&client);
  SyncAdapterClientTestPeer::HandleTick(&client, 4);
  TickLease tick;
  std::string error;
  ASSERT_EQ(client.WaitForTick(0ms, &tick, &error), TickWaitResult::kReceived);
  ASSERT_TRUE(client.FailTick(tick, FaultCode::kInvalidOutput, "bad output",
                              &error));
  EXPECT_EQ(transport->fault.session_id(), 42U);
  EXPECT_EQ(transport->fault.tick_id(), 4U);
  EXPECT_EQ(transport->fault.code(), mujoco_sim::v3::INVALID_OUTPUT);
}

TEST(SyncAdapterClientTest, StopWakesBlockingWaiter) {
  SyncAdapterClient client(Options());
  SyncAdapterClientTestPeer::Configure(&client);
  auto waiting = std::async(std::launch::async, [&] {
    TickLease tick;
    std::string error;
    return client.WaitForTick(std::chrono::milliseconds::max(), &tick, &error);
  });
  std::this_thread::sleep_for(10ms);
  client.Stop();
  EXPECT_EQ(waiting.get(), TickWaitResult::kStopped);
}

static_assert(!std::is_copy_constructible_v<TickLease>);

}  // namespace
