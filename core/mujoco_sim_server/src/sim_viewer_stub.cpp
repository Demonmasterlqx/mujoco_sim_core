#include "sim_viewer.hpp"

#include <exception>
#include <limits>
#include <memory>
#include <utility>

#include <mujoco_sim/core/simulation_core.hpp>

namespace mujoco_sim::server {

struct SimViewer::Impl {};

SimViewer::SnapshotLease::~SnapshotLease() = default;
SimViewer::SnapshotLease::SnapshotLease(
    Impl* owner, std::uint32_t buffer_index,
    std::chrono::steady_clock::time_point capture_started) noexcept
    : owner_(owner), buffer_index_(buffer_index), capture_started_(capture_started) {}
SimViewer::SnapshotLease::SnapshotLease(SnapshotLease&& other) noexcept
    : owner_(std::exchange(other.owner_, nullptr)),
      buffer_index_(other.buffer_index_),
      capture_started_(other.capture_started_) {}
SimViewer::SnapshotLease& SimViewer::SnapshotLease::operator=(SnapshotLease&& other) noexcept {
  owner_ = std::exchange(other.owner_, nullptr);
  buffer_index_ = other.buffer_index_;
  capture_started_ = other.capture_started_;
  return *this;
}
SimViewer::SnapshotLease::operator bool() const noexcept {
  return false;
}
mujoco_sim::core::SimulationSnapshot& SimViewer::SnapshotLease::snapshot() const noexcept {
  std::terminate();
}
void SimViewer::SnapshotLease::Reset(bool) noexcept {
  owner_ = nullptr;
}

SimViewer::SimViewer() : impl_(std::make_unique<Impl>()) {}
SimViewer::~SimViewer() = default;

bool SimViewer::Initialize(std::string* error_message) {
  if (error_message != nullptr) {
    *error_message = "GUI support is disabled in this build";
  }
  return false;
}

bool SimViewer::QueueLoad(std::unique_ptr<mujoco_sim::core::SimulationSnapshot>,
                          std::unique_ptr<mujoco_sim::core::SimulationSnapshot>,
                          std::unique_ptr<mujoco_sim::core::SimulationSnapshot>, const std::string&,
                          std::string*) {
  return false;
}

SimViewer::SnapshotLease SimViewer::TryAcquireSnapshot() noexcept {
  return {};
}

bool SimViewer::TrySubmitSnapshot(SnapshotLease&&) noexcept {
  return false;
}

bool SimViewer::RenderLoop(std::string*) noexcept {
  return true;
}

void SimViewer::RequestScreenshot() noexcept {}
void SimViewer::RequestClose() noexcept {}
bool SimViewer::close_requested() const noexcept {
  return true;
}
bool SimViewer::model_loaded() const {
  return false;
}
SimViewer::IntervalMetrics SimViewer::GetAndResetIntervalMetrics() noexcept {
  return {};
}
void SimViewer::RequestSyncFailureForTesting() noexcept {}
void SimViewer::HoldMailboxForTesting(std::chrono::milliseconds,
                                      std::atomic<bool>* lock_acquired) noexcept {
  if (lock_acquired != nullptr) {
    lock_acquired->store(true, std::memory_order_release);
  }
}
void SimViewer::HoldSyncMetricsForTesting(std::chrono::milliseconds,
                                          std::atomic<bool>* lock_acquired) noexcept {
  if (lock_acquired != nullptr) {
    lock_acquired->store(true, std::memory_order_release);
  }
}
void SimViewer::SetSyncDelayForTesting(std::chrono::milliseconds) noexcept {}
bool SimViewer::sync_active_for_testing() const noexcept {
  return false;
}
double SimViewer::PendingTimeForTesting() const {
  return std::numeric_limits<double>::quiet_NaN();
}
double SimViewer::DisplayTimeForTesting() const {
  return std::numeric_limits<double>::quiet_NaN();
}
bool SimViewer::snapshot_models_isolated_for_testing() const noexcept {
  return false;
}

}  // namespace mujoco_sim::server
