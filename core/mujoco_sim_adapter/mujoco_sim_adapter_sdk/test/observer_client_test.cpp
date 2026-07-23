#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>

#include "../src/internal/observer_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {

class FakeObserverTransport final : public internal::AdapterTransport {
 public:
  bool Register(const mujoco_sim::v4::RegisterAdapterRequest&,
                mujoco_sim::v4::RegisterAdapterResponse*, std::string*) override {
    return false;
  }
  bool Activate(const mujoco_sim::v4::ActivateAdapterRequest&,
                mujoco_sim::v4::ActivateAdapterResponse* response, std::string*) override {
    std::unique_lock lock(mutex_);
    activate_entered_ = true;
    heartbeat_condition_.notify_all();
    heartbeat_condition_.wait(lock, [this] { return !block_activate_; });
    response->set_code(mujoco_sim::v4::OK);
    return true;
  }
  bool Unregister(const mujoco_sim::v4::UnregisterAdapterRequest& request,
                  mujoco_sim::v4::UnregisterAdapterResponse* response, std::string*) override {
    std::lock_guard lock(mutex_);
    ++unregister_calls_;
    last_unregister = request;
    response->set_code(unregister_code);
    return unregister_transport_ok;
  }
  internal::PublishResult PublishCommand(std::string_view) override { return {false, {}}; }
  internal::PublishResult PublishFault(const mujoco_sim::v4::ControllerFault&) override {
    return {false, {}};
  }
  internal::PublishResult PublishHeartbeat(
      const mujoco_sim::v4::SessionHeartbeat& heartbeat) override {
    std::lock_guard lock(mutex_);
    last_heartbeat = heartbeat;
    heartbeat_published_ = true;
    heartbeat_condition_.notify_all();
    if (throw_on_heartbeat_) throw std::runtime_error("injected heartbeat publish failure");
    return {heartbeat_publish_result_, heartbeat_error_};
  }

  void ThrowOnHeartbeat() {
    std::lock_guard lock(mutex_);
    throw_on_heartbeat_ = true;
  }

  void RejectHeartbeat() {
    std::lock_guard lock(mutex_);
    heartbeat_publish_result_ = false;
  }

  void FailHeartbeat(std::string error) {
    std::lock_guard lock(mutex_);
    heartbeat_error_ = std::move(error);
  }

  bool WaitForHeartbeat(std::chrono::milliseconds timeout) {
    std::unique_lock lock(mutex_);
    return heartbeat_condition_.wait_for(lock, timeout, [this] { return heartbeat_published_; });
  }

  void BlockActivate() {
    std::lock_guard lock(mutex_);
    block_activate_ = true;
    activate_entered_ = false;
  }

  bool WaitForActivate(std::chrono::milliseconds timeout) {
    std::unique_lock lock(mutex_);
    return heartbeat_condition_.wait_for(lock, timeout, [this] { return activate_entered_; });
  }

  void ReleaseActivate() {
    {
      std::lock_guard lock(mutex_);
      block_activate_ = false;
    }
    heartbeat_condition_.notify_all();
  }

  std::size_t UnregisterCalls() const {
    std::lock_guard lock(mutex_);
    return unregister_calls_;
  }

  bool unregister_transport_ok = true;
  mujoco_sim::v4::ErrorCode unregister_code = mujoco_sim::v4::OK;
  mujoco_sim::v4::UnregisterAdapterRequest last_unregister;
  mujoco_sim::v4::SessionHeartbeat last_heartbeat;

 private:
  mutable std::mutex mutex_;
  std::condition_variable heartbeat_condition_;
  std::size_t unregister_calls_ = 0;
  bool heartbeat_published_ = false;
  bool throw_on_heartbeat_ = false;
  bool heartbeat_publish_result_ = true;
  bool block_activate_ = false;
  bool activate_entered_ = false;
  std::string heartbeat_error_;
};

class ObserverClientTestPeer {
 public:
  static bool ValidateOptions(ObserverClient* client, std::string* error) {
    return client->impl_->ValidateOptions(error);
  }

  static std::shared_ptr<FakeObserverTransport> Configure(ObserverClient* client,
                                                          std::size_t queue_capacity = 1) {
    auto transport = std::make_shared<FakeObserverTransport>();
    client->impl_->stop_requested_ = false;
    std::lock_guard lock(client->impl_->mutex_);
    client->impl_->options_.queue_capacity = queue_capacity;
    client->impl_->registration_.adapter_id = "watcher";
    client->impl_->registration_.session_id = 42;
    client->impl_->registration_.epoch = 7;
    client->impl_->registration_.layout = ControllerLayout(
        {{"position", {0, 1}, false, 0.0, 0.0}, {"velocity", {1, 1}, false, 0.0, 0.0}}, {}, 2, 0);
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
                          std::uint64_t tick_id = 10, std::array<double, 2> scalars = {1.0, 2.0}) {
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
    ASSERT_TRUE(mujoco_sim::protocol::EncodeDataFrame(header, scalars, &bytes, &error));
    client->impl_->HandleObservationBytes(bytes.data(), bytes.size());
  }

  static mujoco_sim::v4::SessionHeartbeat BuildHeartbeat(ObserverClient* client) {
    std::lock_guard lock(client->impl_->mutex_);
    return client->impl_->BuildHeartbeat();
  }

  static bool Unregister(ObserverClient* client, std::string* error) {
    return client->Unregister(error);
  }

  static bool StartHeartbeatThread(ObserverClient* client) {
    return client->impl_->StartHeartbeatThread();
  }

  static bool StartHeartbeat(ObserverClient* client, std::string* error) {
    return client->impl_->StartHeartbeat(error);
  }

  static void PrepareActivation(ObserverClient* client) {
    std::lock_guard lock(client->impl_->mutex_);
    client->impl_->stop_requested_ = false;
    client->impl_->session_ = internal::AdapterSession{};
    client->impl_->session_.Registered({42, 7});
    client->impl_->session_.StartCompleted();
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
  ASSERT_EQ(client.WaitForObservation(0ms, &first, &error), ObservationWaitResult::kReceived);
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
    return client.WaitForObservation(std::chrono::milliseconds::max(), &frame, &error);
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

TEST(ObserverClientTest, HeartbeatExceptionBecomesTerminalWaitError) {
  ObserverClient client(Options());
  const auto transport = ObserverClientTestPeer::Configure(&client);
  transport->ThrowOnHeartbeat();
  ASSERT_TRUE(ObserverClientTestPeer::StartHeartbeatThread(&client));
  ASSERT_TRUE(transport->WaitForHeartbeat(1s));

  ObservationLease frame;
  std::string error;
  EXPECT_EQ(client.WaitForObservation(1s, &frame, &error), ObservationWaitResult::kError);
  EXPECT_NE(error.find("observer heartbeat thread failed"), std::string::npos) << error;
  EXPECT_NE(error.find("injected heartbeat publish failure"), std::string::npos) << error;
}

TEST(ObserverClientTest, HeartbeatRejectionBecomesNonTerminalDiagnostic) {
  ObserverClient client(Options());
  const auto transport = ObserverClientTestPeer::Configure(&client);
  transport->RejectHeartbeat();
  ASSERT_TRUE(ObserverClientTestPeer::StartHeartbeatThread(&client));
  ASSERT_TRUE(transport->WaitForHeartbeat(1s));

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (client.lifecycle_warning().empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  EXPECT_EQ(client.lifecycle_warning(), "observer heartbeat publish failed; retrying");
  client.Stop();
}

TEST(ObserverClientTest, HeartbeatTransportErrorBecomesTerminalWaitError) {
  ObserverClient client(Options());
  const auto transport = ObserverClientTestPeer::Configure(&client);
  transport->FailHeartbeat("injected heartbeat serialization failure");
  ASSERT_TRUE(ObserverClientTestPeer::StartHeartbeatThread(&client));
  ASSERT_TRUE(transport->WaitForHeartbeat(1s));

  ObservationLease frame;
  std::string error;
  EXPECT_EQ(client.WaitForObservation(1s, &frame, &error), ObservationWaitResult::kError);
  EXPECT_NE(error.find("observer heartbeat publish failed"), std::string::npos) << error;
  EXPECT_NE(error.find("injected heartbeat serialization failure"), std::string::npos) << error;
}

TEST(ObserverClientTest, StopCancelsHeartbeatStartupCommit) {
  ObserverClient client(Options());
  ObserverClientTestPeer::Configure(&client);
  client.Stop();

  std::string error;
  EXPECT_FALSE(ObserverClientTestPeer::StartHeartbeat(&client, &error));
  EXPECT_EQ(error, "observer start was cancelled before heartbeat startup");
}

TEST(ObserverClientTest, StopDuringRemoteActivationCompensatesRegistration) {
  ObserverClient client(Options());
  const auto transport = ObserverClientTestPeer::Configure(&client);
  ObserverClientTestPeer::PrepareActivation(&client);
  transport->BlockActivate();

  std::string activation_error;
  auto activating =
      std::async(std::launch::async, [&] { return client.Activate(&activation_error); });
  ASSERT_TRUE(transport->WaitForActivate(1s));

  client.Stop();
  transport->ReleaseActivate();

  EXPECT_FALSE(activating.get());
  EXPECT_EQ(activation_error, "observer activation was cancelled");
  EXPECT_EQ(transport->UnregisterCalls(), 1U);
  EXPECT_EQ(transport->last_unregister.session_id(), 42U);
  EXPECT_EQ(transport->last_unregister.epoch(), 7U);

  std::string error;
  EXPECT_TRUE(client.Unregister(&error)) << error;
  EXPECT_EQ(transport->UnregisterCalls(), 1U);
}

static_assert(!std::is_copy_constructible_v<ObservationLease>);

}  // namespace
