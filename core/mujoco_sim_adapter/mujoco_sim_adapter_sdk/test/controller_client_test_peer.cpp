#include "../src/internal/controller_client_test_peer.hpp"

#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <mujoco_sim/protocol/data_frame.hpp>

#include "../src/internal/controller_client_impl.hpp"

namespace mujoco_sim::adapter_sdk {

void ControllerClientTestPeer::Configure(ControllerClient& client,
                                         std::shared_ptr<internal::AdapterTransport> transport,
                                         std::chrono::milliseconds keepalive) {
  ConfigureWithLayout(client, std::move(transport),
                      ControllerLayout({{"position", {0, 1}, false, 0.0, 0.0},
                                        {"velocity", {1, 1}, false, 0.0, 0.0}},
                                       {{"actuator", {0, 2}, true, -10.0, 10.0}}, 2, 2),
                      keepalive);
}

void ControllerClientTestPeer::ConfigureWithLayout(
    ControllerClient& client, std::shared_ptr<internal::AdapterTransport> transport,
    ControllerLayout layout, std::chrono::milliseconds keepalive) {
  auto& impl = *client.impl_;
  std::lock_guard lock(impl.mutex);
  impl.stop = false;
  impl.started = true;
  impl.active = true;
  impl.remote_registered = false;
  impl.unregistered = false;
  impl.waiting = false;
  impl.seen_status = false;
  impl.registration.adapter_id = "controller";
  impl.registration.session_id = 42;
  impl.registration.epoch = 7;
  impl.registration.layout = std::move(layout);
  impl.fingerprint = {11, 12};
  impl.keepalive = keepalive;
  impl.transport = std::move(transport);
  impl.latest.reset();
  impl.latest_command.clear();
  impl.generation = 0;
  impl.next_sequence = 1;
  impl.last_read = 0;
  impl.terminal.clear();
}

void ControllerClientTestPeer::HandleState(ControllerClient& client, std::uint64_t sequence,
                                           std::uint64_t tick_id,
                                           std::span<const double> observations) {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = mujoco_sim::protocol::DataFrameKind::kControllerState;
  header.session_id = 42;
  header.epoch = 7;
  header.tick_id = tick_id;
  header.layout = {11, 12};
  header.sequence = sequence;
  header.simulation_time_s = 0.25;
  std::string bytes;
  mujoco_sim::protocol::DataFrameError error;
  if (mujoco_sim::protocol::EncodeDataFrame(header, observations, &bytes, &error)) {
    client.impl_->OnState(bytes.data(), bytes.size());
  }
}

void ControllerClientTestPeer::HandleStatus(ControllerClient& client,
                                            const mujoco_sim::v4::ServerStatus& status) {
  const auto bytes = status.SerializeAsString();
  client.impl_->OnStatus(bytes.data(), bytes.size());
}

void ControllerClientTestPeer::StartPublisher(ControllerClient& client) {
  auto& impl = *client.impl_;
  auto lifecycle_operation = impl.lifecycle.Acquire();
  if (!impl.publisher.joinable()) {
    impl.publisher = std::thread([&impl] { impl.RunPublishLoop(); });
  }
}

void ControllerClientTestPeer::EnableRemoteRegistration(ControllerClient& client) {
  auto& impl = *client.impl_;
  auto lifecycle_operation = impl.lifecycle.Acquire();
  impl.wire.set_session_id(impl.registration.session_id);
  impl.wire.set_epoch(impl.registration.epoch);
  impl.remote_registered = true;
  impl.unregistered = false;
}

void ControllerClientTestPeer::FailPublisherStart(ControllerClient& client) {
  auto& impl = *client.impl_;
  auto lifecycle_operation = impl.lifecycle.Acquire();
  impl.before_publisher_start = [] {
    throw std::runtime_error("injected publisher thread creation failure");
  };
}

bool ControllerClientTestPeer::IsWaiting(ControllerClient& client) {
  std::lock_guard lock(client.impl_->mutex);
  return client.impl_->waiting;
}

void ControllerClientTestPeer::PrepareActivation(ControllerClient& client) {
  auto& impl = *client.impl_;
  auto lifecycle_operation = impl.lifecycle.Acquire();
  std::lock_guard lock(impl.mutex);
  impl.started = true;
  impl.active = false;
  impl.stop = false;
}

bool ControllerClientTestPeer::StopRequested(ControllerClient& client) {
  return client.impl_->stop.load();
}

}  // namespace mujoco_sim::adapter_sdk
