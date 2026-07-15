#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>

#include "../src/internal/observer_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {

class FakeObserverTransport final : public internal::AdapterTransport {
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
  bool Unregister(const mujoco_sim::v3::UnregisterAdapterRequest& request,
                  mujoco_sim::v3::UnregisterAdapterResponse* response,
                  std::string*) override {
    last_unregister = request;
    response->set_code(unregister_code);
    return unregister_transport_ok;
  }
  bool PublishCommit(std::string_view) override { return false; }
  bool PublishFault(const mujoco_sim::v3::ControllerFault&) override {
    return false;
  }
  void PublishHeartbeat(
      const mujoco_sim::v3::SessionHeartbeat& heartbeat) override {
    last_heartbeat = heartbeat;
  }

  bool unregister_transport_ok = true;
  mujoco_sim::v3::ErrorCode unregister_code = mujoco_sim::v3::OK;
  mujoco_sim::v3::UnregisterAdapterRequest last_unregister;
  mujoco_sim::v3::SessionHeartbeat last_heartbeat;
};

class ObserverClientTestPeer {
 public:
  static bool ValidateOptions(ObserverClient* client, std::string* error) {
    return client->impl_->ValidateOptions(error);
  }

  static std::shared_ptr<FakeObserverTransport> Configure(
      ObserverClient* client, std::size_t queue_capacity = 1) {
    auto transport = std::make_shared<FakeObserverTransport>();
    client->impl_->stop_requested_ = false;
    std::lock_guard lock(client->impl_->mutex_);
    client->impl_->options_.queue_capacity = queue_capacity;
    client->impl_->registration_.adapter_id = "watcher";
    client->impl_->registration_.session_id = 42;
    client->impl_->registration_.epoch = 7;
    client->impl_->registration_.layout = ControllerLayout(
        {{"position", {0, 1}, false, 0.0, 0.0},
         {"velocity", {1, 1}, false, 0.0, 0.0}},
        {}, 2, 0);
    client->impl_->layout_fingerprint_ = {11, 12};
    client->impl_->session_ = internal::AdapterSession{};
    client->impl_->session_.Registered({42, 7});
    client->impl_->session_.StartCompleted();
    client->impl_->session_.BeginActivation();
    client->impl_->session_.Activated();
    client->impl_->monitor_ecal_ = false;
    client->impl_->observations_.Clear();
    client->impl_->last_sequence_ = 0;
    client->impl_->last_tick_id_ = 0;
    client->impl_->dropped_frames_ = 0;
    client->impl_->terminal_error_.clear();
    client->impl_->transport_ = transport;
    return transport;
  }

  static void HandleFrame(ObserverClient* client, std::uint64_t sequence,
                          std::uint64_t tick_id = 10,
                          std::array<double, 2> scalars = {1.0, 2.0}) {
    mujoco_sim::protocol::DataFrameHeader header;
    header.kind = mujoco_sim::protocol::DataFrameKind::kObservation;
    header.session_id = 42;
    header.epoch = 7;
    header.tick_id = tick_id;
    header.layout = {11, 12};
    header.sequence = sequence;
    header.simulation_time_s = 0.25;
    std::string bytes;
    mujoco_sim::protocol::DataFrameError error;
    ASSERT_TRUE(mujoco_sim::protocol::EncodeDataFrame(
        header, scalars, &bytes, &error));
    client->impl_->HandleObservationBytes(bytes.data(), bytes.size());
  }

  static mujoco_sim::v3::SessionHeartbeat BuildHeartbeat(
      ObserverClient* client) {
    std::lock_guard lock(client->impl_->mutex_);
    return client->impl_->BuildHeartbeat();
  }

  static bool Unregister(ObserverClient* client, std::string* error) {
    return client->Unregister(error);
  }
};

}  // namespace mujoco_sim::adapter_sdk

namespace {

using namespace std::chrono_literals;
using mujoco_sim::adapter_sdk::ObservationLease;
using mujoco_sim::adapter_sdk::ObservationWaitResult;
using mujoco_sim::adapter_sdk::ObserverClient;
using mujoco_sim::adapter_sdk::ObserverClientTestPeer;
using mujoco_sim::adapter_sdk::ObserverOptions;

ObserverOptions Options() {
  ObserverOptions options;
  options.observer_id = "watcher";
  options.observation_names = {"position", "velocity"};
  return options;
}

TEST(ObserverClientTest, DefaultCadenceIsExplicitOneHundredHertz) {
  const auto options = Options();
  EXPECT_FALSE(options.every_n_steps.has_value());
  ASSERT_TRUE(options.period_s.has_value());
  EXPECT_DOUBLE_EQ(*options.period_s, 0.01);
}

TEST(ObserverClientTest, ValidatesCadenceAndHeartbeatRatio) {
  std::string error;
  auto options = Options();
  ObserverClient valid(options);
  EXPECT_TRUE(ObserverClientTestPeer::ValidateOptions(&valid, &error)) << error;

  options.every_n_steps = 1;
  ObserverClient both(options);
  EXPECT_FALSE(ObserverClientTestPeer::ValidateOptions(&both, &error));

  options = Options();
  options.heartbeat_period_ms = 400;
  ObserverClient slow(options);
  EXPECT_FALSE(ObserverClientTestPeer::ValidateOptions(&slow, &error));
}

TEST(ObserverClientTest, DeliversDenseLeaseAndCountsDrops) {
  ObserverClient client(Options());
  ObserverClientTestPeer::Configure(&client, 2);
  ObserverClientTestPeer::HandleFrame(&client, 1);
  ObserverClientTestPeer::HandleFrame(&client, 3);
  ObserverClientTestPeer::HandleFrame(&client, 4);

  ObservationLease first;
  std::string error;
  ASSERT_EQ(client.WaitForObservation(0ms, &first, &error),
            ObservationWaitResult::kReceived);
  EXPECT_EQ(first.sequence(), 3U);
  EXPECT_EQ(first.tick_id(), 10U);
  ASSERT_EQ(first.observations().size(), 2U);
  EXPECT_DOUBLE_EQ(first.observations()[1], 2.0);
  EXPECT_EQ(client.dropped_frames(), 2U);
}

TEST(ObserverClientTest, StopWakesBlockingWaiter) {
  ObserverClient client(Options());
  ObserverClientTestPeer::Configure(&client);
  auto waiting = std::async(std::launch::async, [&] {
    ObservationLease frame;
    std::string error;
    return client.WaitForObservation(std::chrono::milliseconds::max(),
                                     &frame, &error);
  });
  std::this_thread::sleep_for(10ms);
  client.Stop();
  EXPECT_EQ(waiting.get(), ObservationWaitResult::kStopped);
}

TEST(ObserverClientTest, HeartbeatTracksLatestConsumedSequence) {
  ObserverClient client(Options());
  ObserverClientTestPeer::Configure(&client);
  ObserverClientTestPeer::HandleFrame(&client, 9, 15);
  const auto heartbeat = ObserverClientTestPeer::BuildHeartbeat(&client);
  EXPECT_EQ(heartbeat.session_id(), 42U);
  EXPECT_EQ(heartbeat.epoch(), 7U);
  EXPECT_EQ(heartbeat.last_seen(), 9U);
}

TEST(ObserverClientTest, FailedUnregisterPreservesSessionForRetry) {
  ObserverClient client(Options());
  const auto transport = ObserverClientTestPeer::Configure(&client);
  transport->unregister_transport_ok = false;
  std::string error;
  EXPECT_FALSE(ObserverClientTestPeer::Unregister(&client, &error));
  transport->unregister_transport_ok = true;
  EXPECT_TRUE(ObserverClientTestPeer::Unregister(&client, &error)) << error;
  EXPECT_EQ(transport->last_unregister.session_id(), 42U);
  EXPECT_EQ(transport->last_unregister.epoch(), 7U);
}

static_assert(!std::is_copy_constructible_v<ObservationLease>);

}  // namespace
