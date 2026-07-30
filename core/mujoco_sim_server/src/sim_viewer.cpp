#include "sim_viewer.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <mujoco_sim/core/simulation_core.hpp>

#include "glfw_adapter.h"
#include "simulate.h"

namespace mujoco_sim::server {
namespace {

using Clock = std::chrono::steady_clock;

std::uint64_t ToNanoseconds(Clock::duration duration) noexcept {
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  return nanoseconds > 0 ? static_cast<std::uint64_t>(nanoseconds) : 0;
}

void UpdateMaximum(std::atomic<std::uint64_t>* maximum, std::uint64_t value) noexcept {
  auto observed = maximum->load(std::memory_order_relaxed);
  while (observed < value &&
         !maximum->compare_exchange_weak(observed, value, std::memory_order_relaxed,
                                         std::memory_order_relaxed)) {
  }
}

struct AtomicDurationMetrics {
  void Record(Clock::duration duration) noexcept {
    const auto nanoseconds = ToNanoseconds(duration);
    total_ns.fetch_add(nanoseconds, std::memory_order_relaxed);
    sample_count.fetch_add(1, std::memory_order_relaxed);
    UpdateMaximum(&max_ns, nanoseconds);
  }

  std::atomic<std::uint64_t> total_ns{0};
  std::atomic<std::uint64_t> max_ns{0};
  std::atomic<std::uint64_t> sample_count{0};
};

struct DurationMetrics {
  void Record(Clock::duration duration) noexcept {
    const auto nanoseconds = ToNanoseconds(duration);
    total_ns += nanoseconds;
    max_ns = std::max(max_ns, nanoseconds);
    ++sample_count;
  }

  void Reset() noexcept {
    total_ns = 0;
    max_ns = 0;
    sample_count = 0;
  }

  std::uint64_t total_ns = 0;
  std::uint64_t max_ns = 0;
  std::uint64_t sample_count = 0;
};

}  // namespace

bool detail::ExtractViewerBodyWrenches(
    const mjModel_* model, const mjData_* data,
    std::vector<mujoco_sim::core::InteractiveBodyWrench>* output) {
  if (model == nullptr || data == nullptr || output == nullptr) return false;
  for (int body_id = 1; body_id < model->nbody; ++body_id) {
    const mjtNum* source = data->xfrc_applied + 6 * body_id;
    const bool finite = std::all_of(
        source, source + 6, [](mjtNum value) { return std::isfinite(static_cast<double>(value)); });
    if (!finite) return false;
  }

  output->reserve(model->nbody > 0 ? static_cast<std::size_t>(model->nbody - 1) : 0U);
  output->clear();
  for (int body_id = 1; body_id < model->nbody; ++body_id) {
    const mjtNum* source = data->xfrc_applied + 6 * body_id;
    const bool nonzero = std::any_of(source, source + 6,
                                     [](mjtNum value) { return value != static_cast<mjtNum>(0); });
    if (!nonzero || model->body_dofnum[body_id] <= 0) continue;
    output->push_back({
        .body_id = body_id,
        .force = {static_cast<double>(source[0]), static_cast<double>(source[1]),
                  static_cast<double>(source[2])},
        .torque = {static_cast<double>(source[3]), static_cast<double>(source[4]),
                   static_cast<double>(source[5])},
    });
  }
  return true;
}

namespace {

detail::ViewerSnapshotMetadata DescribeSnapshot(
    const std::unique_ptr<mujoco_sim::core::SimulationSnapshot>& snapshot) noexcept {
  if (snapshot == nullptr || snapshot->model() == nullptr || snapshot->data() == nullptr) {
    return {};
  }
  return {
      .model_identity = snapshot->model(),
      .data_identity = snapshot->data(),
      .model_signature = snapshot->model()->signature,
      .data_signature = snapshot->data()->signature,
      .model_buffer_bytes = static_cast<std::uint64_t>(snapshot->model()->nbuffer),
      .data_buffer_bytes = static_cast<std::uint64_t>(snapshot->data()->nbuffer),
  };
}

}  // namespace

struct SimViewer::Impl {
  static constexpr std::uint32_t kFirstBuffer = 0;
  static constexpr std::uint32_t kSecondBuffer = 1;
  static constexpr std::uint32_t kAllBuffersFree = 0b11;

  explicit Impl(InputOptions options) : input_options(options) {}

  struct BufferRelease {
    ~BufferRelease() {
      if (owner != nullptr) {
        owner->ReleaseBuffer(index);
      }
    }

    Impl* owner = nullptr;
    std::uint32_t index = 0;
  };

  struct SyncActivity {
    explicit SyncActivity(std::atomic<bool>* active_flag) : active(active_flag) {
      active->store(true, std::memory_order_release);
    }

    ~SyncActivity() { active->store(false, std::memory_order_release); }

    std::atomic<bool>* active;
  };

  void ReleaseBuffer(std::uint32_t index) noexcept {
    free_buffers.fetch_or(std::uint32_t{1} << index, std::memory_order_release);
  }

  void RecordCapture(Clock::time_point started) noexcept {
    capture_duration.Record(Clock::now() - started);
  }

  void ReleasePendingLocked() noexcept {
    if (pending_buffer.has_value()) {
      ReleaseBuffer(*pending_buffer);
      pending_buffer.reset();
    }
  }

  void RecordWorkerFailure(const char* message) noexcept {
    worker_failed.store(true, std::memory_order_release);
    try {
      std::lock_guard metrics_lock(sync_metrics_mutex);
      ++sync_failures;
    } catch (...) {
    }
    try {
      worker_failure = "viewer sync worker failed: ";
      worker_failure.append(message == nullptr ? "unknown error" : message);
    } catch (...) {
    }
    stop_requested.store(true, std::memory_order_release);
    {
      std::lock_guard lock(mailbox_mutex);
      accepting_submissions.store(false, std::memory_order_release);
      ReleasePendingLocked();
    }
    if (simulate != nullptr) {
      simulate->exitrequest.store(1);
    }
    mailbox_condition.notify_all();
  }

  bool SnapshotStorageValid() const noexcept { return snapshot_storage_valid; }

  void PublishInput(std::span<const mujoco_sim::core::InteractiveActuatorWrite> control_writes,
                    const ViewerWrenchFrame* wrench_frame) {
    if (control_writes.empty() && wrench_frame == nullptr) return;
    std::lock_guard lock(input_mutex);
    if (!accepting_input) return;
    if (input_options.control_enabled) {
      for (const auto& write : control_writes) {
        if (write.actuator_id >= 0 &&
            static_cast<std::size_t>(write.actuator_id) < pending_control_values.size()) {
          pending_control_values[write.actuator_id] = write.value;
        }
      }
    }
    if (input_options.mouse_force_enabled && wrench_frame != nullptr) {
      latest_wrench_sequence = wrench_frame->sequence;
      latest_wrench_generated_at = wrench_frame->generated_at;
      latest_wrench_values.assign(wrench_frame->body_wrenches.begin(),
                                  wrench_frame->body_wrenches.end());
      wrench_pending = true;
    }
  }

  void ExtractInputLocked() {
    extracted_control_writes.clear();
    extracted_wrench_frame_valid = false;
    if (input_options.control_enabled) {
      for (int actuator_id = 0; actuator_id < display_snapshot->model()->nu; ++actuator_id) {
        const mjtNum before = control_before_sync[actuator_id];
        const mjtNum after = display_snapshot->data()->ctrl[actuator_id];
        if (before != after) {
          extracted_control_writes.push_back(
              {.actuator_id = actuator_id, .value = static_cast<double>(after)});
        }
      }
    }
    if (!input_options.mouse_force_enabled) return;

    if (!detail::ExtractViewerBodyWrenches(display_snapshot->model(), display_snapshot->data(),
                                           &extracted_wrenches)) {
      throw std::runtime_error("viewer produced a non-finite mouse wrench");
    }
    const bool nonempty = !extracted_wrenches.empty();
    if (nonempty || previous_wrench_nonempty) {
      extracted_wrench_frame.sequence = ++wrench_sequence;
      extracted_wrench_frame.generated_at = Clock::now();
      extracted_wrench_frame.body_wrenches.assign(extracted_wrenches.begin(),
                                                  extracted_wrenches.end());
      extracted_wrench_frame_valid = true;
    }
    previous_wrench_nonempty = nonempty;
  }

  void SynchronizeBuffer(std::uint32_t index) {
    if (fail_next_sync.exchange(false, std::memory_order_acq_rel)) {
      throw std::runtime_error("injected viewer sync worker failure");
    }
    if (!SnapshotStorageValid() || simulate == nullptr) {
      throw std::runtime_error("viewer snapshot storage is invalid");
    }

    const auto started = Clock::now();
    bool completed = false;
    {
      const mujoco::MutexLock viewer_lock(simulate->mtx);
      SyncActivity activity(&sync_active);
      const auto delay_ns = sync_delay_ns.load(std::memory_order_acquire);
      if (delay_ns != 0) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(delay_ns));
      }
      if (!stop_requested.load(std::memory_order_acquire) && simulate->exitrequest.load() == 0 &&
          simulate->loadrequest == 0 && simulate->m_ != nullptr) {
        mj_copyData(display_snapshot->data(), display_snapshot->model(),
                    production_snapshots[index]->data());
        if (input_options.control_enabled && !control_before_sync.empty()) {
          mju_copy(control_before_sync.data(), display_snapshot->data()->ctrl,
                   display_snapshot->model()->nu);
        }
        simulate->Sync();
        ExtractInputLocked();
        completed = true;
      }
    }
    if (completed) {
      PublishInput(extracted_control_writes,
                   extracted_wrench_frame_valid ? &extracted_wrench_frame : nullptr);
      std::lock_guard metrics_lock(sync_metrics_mutex);
      sync_duration.Record(Clock::now() - started);
      ++sync_completed;
    }
  }

  void WorkerMain() {
    while (true) {
      std::optional<std::uint32_t> buffer;
      {
        std::unique_lock lock(mailbox_mutex);
        mailbox_condition.wait(lock, [this] {
          return pending_buffer.has_value() || stop_requested.load(std::memory_order_acquire);
        });
        if (stop_requested.load(std::memory_order_acquire)) {
          ReleasePendingLocked();
          return;
        }
        buffer = pending_buffer;
        pending_buffer.reset();
      }

      BufferRelease release{this, *buffer};
      SynchronizeBuffer(*buffer);
    }
  }

  void WorkerEntry() noexcept {
    try {
      WorkerMain();
    } catch (const std::exception& error) {
      RecordWorkerFailure(error.what());
    } catch (...) {
      RecordWorkerFailure("non-standard exception");
    }
  }

  mjvCamera camera{};
  mjvOption option{};
  mjvPerturb perturb{};
  std::unique_ptr<mujoco::Simulate> simulate;
  InputOptions input_options;
  std::unique_ptr<mujoco_sim::core::SimulationSnapshot> display_snapshot;
  std::array<std::unique_ptr<mujoco_sim::core::SimulationSnapshot>, 2> production_snapshots;
  bool snapshot_storage_valid = false;

  std::vector<mjtNum> control_before_sync;
  std::vector<mujoco_sim::core::InteractiveActuatorWrite> extracted_control_writes;
  std::vector<mujoco_sim::core::InteractiveBodyWrench> extracted_wrenches;
  ViewerWrenchFrame extracted_wrench_frame;
  bool extracted_wrench_frame_valid = false;
  bool previous_wrench_nonempty = false;
  std::uint64_t wrench_sequence = 0;

  std::mutex input_mutex;
  std::vector<std::optional<double>> pending_control_values;
  std::uint64_t latest_wrench_sequence = 0;
  Clock::time_point latest_wrench_generated_at{};
  std::vector<mujoco_sim::core::InteractiveBodyWrench> latest_wrench_values;
  bool wrench_pending = false;
  bool accepting_input = false;

  std::mutex mailbox_mutex;
  std::condition_variable mailbox_condition;
  std::optional<std::uint32_t> pending_buffer;
  std::atomic<std::uint32_t> free_buffers{0};
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> accepting_submissions{false};

  std::jthread sync_worker;
  std::string worker_failure;
  std::atomic<bool> worker_failed{false};
  std::atomic<bool> fail_next_sync{false};
  std::atomic<bool> sync_active{false};
  std::atomic<std::uint64_t> sync_delay_ns{0};
  bool glfw_initialized = false;

  std::atomic<std::uint64_t> capture_attempts{0};
  std::atomic<std::uint64_t> capture_submitted{0};
  std::atomic<std::uint64_t> capture_skipped{0};
  std::atomic<std::uint64_t> pending_replaced{0};
  AtomicDurationMetrics capture_duration;

  std::mutex sync_metrics_mutex;
  std::uint64_t sync_completed = 0;
  std::uint64_t sync_failures = 0;
  DurationMetrics sync_duration;
};

SimViewer::SnapshotLease::SnapshotLease(Impl* owner, std::uint32_t buffer_index,
                                        Clock::time_point capture_started) noexcept
    : owner_(owner), buffer_index_(buffer_index), capture_started_(capture_started) {}

SimViewer::SnapshotLease::~SnapshotLease() {
  Reset(true);
}

SimViewer::SnapshotLease::SnapshotLease(SnapshotLease&& other) noexcept
    : owner_(std::exchange(other.owner_, nullptr)),
      buffer_index_(other.buffer_index_),
      capture_started_(other.capture_started_) {}

SimViewer::SnapshotLease& SimViewer::SnapshotLease::operator=(SnapshotLease&& other) noexcept {
  if (this != &other) {
    Reset(true);
    owner_ = std::exchange(other.owner_, nullptr);
    buffer_index_ = other.buffer_index_;
    capture_started_ = other.capture_started_;
  }
  return *this;
}

SimViewer::SnapshotLease::operator bool() const noexcept {
  return owner_ != nullptr;
}

mujoco_sim::core::SimulationSnapshot& SimViewer::SnapshotLease::snapshot() const noexcept {
  return *owner_->production_snapshots[buffer_index_];
}

void SimViewer::SnapshotLease::Reset(bool skipped) noexcept {
  if (owner_ == nullptr) {
    return;
  }
  owner_->RecordCapture(capture_started_);
  if (skipped) {
    owner_->capture_skipped.fetch_add(1, std::memory_order_relaxed);
  }
  owner_->ReleaseBuffer(buffer_index_);
  owner_ = nullptr;
}

SimViewer::SimViewer() : SimViewer(InputOptions{}) {}

SimViewer::SimViewer(InputOptions options) : impl_(std::make_unique<Impl>(options)) {}

SimViewer::~SimViewer() {
  RequestClose();
  if (impl_->sync_worker.joinable()) {
    impl_->sync_worker.join();
  }
  impl_->simulate.reset();
  if (impl_->glfw_initialized) {
    glfwTerminate();
    impl_->glfw_initialized = false;
  }
}

bool SimViewer::Initialize(std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  if (glfwInit() != GLFW_TRUE) {
    const char* description = nullptr;
    glfwGetError(&description);
    *error_message = description == nullptr
                         ? "failed to initialize GLFW"
                         : "failed to initialize GLFW: " + std::string(description);
    return false;
  }
  impl_->glfw_initialized = true;

  try {
    mjv_defaultCamera(&impl_->camera);
    mjv_defaultOption(&impl_->option);
    mjv_defaultPerturb(&impl_->perturb);
    impl_->simulate =
        std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &impl_->camera,
                                           &impl_->option, &impl_->perturb, true);
  } catch (const std::exception& error) {
    *error_message = "failed to create MuJoCo viewer: " + std::string(error.what());
    return false;
  } catch (...) {
    *error_message = "failed to create MuJoCo viewer: non-standard exception";
    return false;
  }
  return true;
}

bool SimViewer::QueueLoad(
    std::unique_ptr<mujoco_sim::core::SimulationSnapshot> display_snapshot,
    std::unique_ptr<mujoco_sim::core::SimulationSnapshot> first_production_snapshot,
    std::unique_ptr<mujoco_sim::core::SimulationSnapshot> second_production_snapshot,
    const std::string& displayed_filename, std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  if (impl_->simulate == nullptr ||
      !detail::ViewerSnapshotsCompatible(DescribeSnapshot(display_snapshot),
                                         DescribeSnapshot(first_production_snapshot),
                                         DescribeSnapshot(second_production_snapshot))) {
    *error_message =
        "viewer requires an isolated display model, independent data buffers, and two "
        "signature-compatible production snapshots";
    return false;
  }

  {
    std::lock_guard lock(impl_->mailbox_mutex);
    if (impl_->display_snapshot != nullptr) {
      *error_message = "viewer snapshots have already been configured";
      return false;
    }
    impl_->display_snapshot = std::move(display_snapshot);
    impl_->production_snapshots[Impl::kFirstBuffer] = std::move(first_production_snapshot);
    impl_->production_snapshots[Impl::kSecondBuffer] = std::move(second_production_snapshot);
    impl_->snapshot_storage_valid = true;
    impl_->free_buffers.store(Impl::kAllBuffersFree, std::memory_order_release);
    impl_->accepting_submissions.store(true, std::memory_order_release);
  }

  {
    std::lock_guard input_lock(impl_->input_mutex);
    const auto* model = impl_->display_snapshot->model();
    if (impl_->input_options.control_enabled) {
      impl_->control_before_sync.resize(static_cast<std::size_t>(model->nu));
      impl_->extracted_control_writes.reserve(static_cast<std::size_t>(model->nu));
      impl_->pending_control_values.resize(static_cast<std::size_t>(model->nu));
    }
    if (impl_->input_options.mouse_force_enabled) {
      const auto body_capacity = model->nbody > 0 ? static_cast<std::size_t>(model->nbody - 1) : 0U;
      impl_->extracted_wrenches.reserve(body_capacity);
      impl_->extracted_wrench_frame.body_wrenches.reserve(body_capacity);
      impl_->latest_wrench_values.reserve(body_capacity);
    }
    impl_->accepting_input = true;
  }

  const mujoco::MutexLock viewer_lock(impl_->simulate->mtx);
  impl_->simulate->mnew_ = const_cast<mjModel_*>(impl_->display_snapshot->model());
  impl_->simulate->dnew_ = impl_->display_snapshot->data();
  std::snprintf(impl_->simulate->filename, sizeof(impl_->simulate->filename), "%s",
                displayed_filename.c_str());
  impl_->simulate->loadrequest = 2;
  return true;
}

SimViewer::SnapshotLease SimViewer::TryAcquireSnapshot() noexcept {
  impl_->capture_attempts.fetch_add(1, std::memory_order_relaxed);
  std::unique_lock lock(impl_->mailbox_mutex, std::try_to_lock);
  if (!lock.owns_lock() || !impl_->accepting_submissions.load(std::memory_order_acquire)) {
    impl_->capture_skipped.fetch_add(1, std::memory_order_relaxed);
    return {};
  }

  const auto free = impl_->free_buffers.load(std::memory_order_acquire);
  if (free == 0) {
    impl_->capture_skipped.fetch_add(1, std::memory_order_relaxed);
    return {};
  }
  const auto index = (free & 1U) != 0 ? Impl::kFirstBuffer : Impl::kSecondBuffer;
  impl_->free_buffers.fetch_and(~(std::uint32_t{1} << index), std::memory_order_acq_rel);
  return SnapshotLease(impl_.get(), index, Clock::now());
}

bool SimViewer::TrySubmitSnapshot(SnapshotLease&& lease) noexcept {
  if (lease.owner_ != impl_.get()) {
    lease.Reset(true);
    return false;
  }

  std::unique_lock lock(impl_->mailbox_mutex, std::try_to_lock);
  if (!lock.owns_lock() || !impl_->accepting_submissions.load(std::memory_order_acquire)) {
    lease.Reset(true);
    return false;
  }

  impl_->RecordCapture(lease.capture_started_);
  if (impl_->pending_buffer.has_value()) {
    impl_->ReleaseBuffer(*impl_->pending_buffer);
    impl_->pending_replaced.fetch_add(1, std::memory_order_relaxed);
  }
  impl_->pending_buffer = lease.buffer_index_;
  lease.owner_ = nullptr;
  impl_->capture_submitted.fetch_add(1, std::memory_order_relaxed);
  lock.unlock();
  impl_->mailbox_condition.notify_one();
  return true;
}

bool SimViewer::TryConsumeInput(ViewerInputBatch* output) noexcept {
  if (output == nullptr) return false;
  std::unique_lock lock(impl_->input_mutex, std::try_to_lock);
  if (!lock.owns_lock() || !impl_->accepting_input) return false;

  const auto control_count = static_cast<std::size_t>(
      std::count_if(impl_->pending_control_values.begin(), impl_->pending_control_values.end(),
                    [](const auto& value) { return value.has_value(); }));
  if (control_count == 0 && !impl_->wrench_pending) return false;
  try {
    output->actuator_writes.reserve(control_count);
    if (impl_->wrench_pending) {
      if (!output->wrench_frame) output->wrench_frame.emplace();
      output->wrench_frame->body_wrenches.reserve(impl_->latest_wrench_values.size());
    }
  } catch (...) {
    return false;
  }

  output->actuator_writes.clear();
  for (std::size_t actuator_id = 0; actuator_id < impl_->pending_control_values.size();
       ++actuator_id) {
    auto& value = impl_->pending_control_values[actuator_id];
    if (!value) continue;
    output->actuator_writes.push_back(
        {.actuator_id = static_cast<int>(actuator_id), .value = *value});
    value.reset();
  }
  if (impl_->wrench_pending) {
    output->wrench_frame->sequence = impl_->latest_wrench_sequence;
    output->wrench_frame->generated_at = impl_->latest_wrench_generated_at;
    output->wrench_frame->body_wrenches.assign(impl_->latest_wrench_values.begin(),
                                               impl_->latest_wrench_values.end());
    impl_->wrench_pending = false;
  } else {
    output->wrench_frame.reset();
  }
  return true;
}

bool SimViewer::RenderLoop(std::string* error_message) noexcept {
  if (error_message == nullptr) {
    return false;
  }
  error_message->clear();
  if (impl_->simulate == nullptr || impl_->display_snapshot == nullptr) {
    *error_message = "viewer is not initialized with snapshot storage";
    return false;
  }

  try {
    impl_->sync_worker = std::jthread([impl = impl_.get()] { impl->WorkerEntry(); });
    impl_->simulate->RenderLoop();
  } catch (const std::exception& error) {
    *error_message = "viewer render loop failed: " + std::string(error.what());
  } catch (...) {
    *error_message = "viewer render loop failed: non-standard exception";
  }

  RequestClose();
  if (impl_->sync_worker.joinable()) {
    impl_->sync_worker.join();
  }
  if (impl_->worker_failed.load(std::memory_order_acquire)) {
    *error_message =
        impl_->worker_failure.empty() ? "viewer sync worker failed" : impl_->worker_failure;
  }
  return error_message->empty();
}

void SimViewer::RequestScreenshot() noexcept {
  if (impl_->simulate != nullptr) {
    impl_->simulate->screenshotrequest.store(1);
  }
}

void SimViewer::RequestClose() noexcept {
  impl_->stop_requested.store(true, std::memory_order_release);
  impl_->accepting_submissions.store(false, std::memory_order_release);
  try {
    std::lock_guard input_lock(impl_->input_mutex);
    impl_->accepting_input = false;
    for (auto& value : impl_->pending_control_values)
      value.reset();
    impl_->wrench_pending = false;
  } catch (...) {
  }
  impl_->mailbox_condition.notify_all();
  if (impl_->simulate != nullptr) {
    impl_->simulate->exitrequest.store(1);
  }
}

bool SimViewer::close_requested() const noexcept {
  return impl_->stop_requested.load(std::memory_order_acquire) ||
         (impl_->simulate != nullptr && impl_->simulate->exitrequest.load() != 0);
}

bool SimViewer::model_loaded() const {
  if (impl_->simulate == nullptr || close_requested()) {
    return false;
  }
  const mujoco::MutexLock viewer_lock(impl_->simulate->mtx);
  return impl_->simulate->loadrequest == 0 && impl_->simulate->m_ != nullptr;
}

SimViewer::IntervalMetrics SimViewer::GetAndResetIntervalMetrics() noexcept {
  IntervalMetrics output;
  output.capture_attempts = impl_->capture_attempts.exchange(0, std::memory_order_relaxed);
  output.capture_submitted = impl_->capture_submitted.exchange(0, std::memory_order_relaxed);
  output.capture_skipped = impl_->capture_skipped.exchange(0, std::memory_order_relaxed);
  output.pending_replaced = impl_->pending_replaced.exchange(0, std::memory_order_relaxed);

  const auto capture_total = impl_->capture_duration.total_ns.exchange(0);
  const auto capture_count = impl_->capture_duration.sample_count.exchange(0);
  const auto capture_max = impl_->capture_duration.max_ns.exchange(0);
  constexpr double kNanosecondsPerMicrosecond = 1000.0;
  output.capture_avg_us = capture_count == 0
                              ? 0.0
                              : static_cast<double>(capture_total) /
                                    static_cast<double>(capture_count) / kNanosecondsPerMicrosecond;
  output.capture_max_us = static_cast<double>(capture_max) / kNanosecondsPerMicrosecond;

  std::unique_lock sync_metrics_lock(impl_->sync_metrics_mutex, std::try_to_lock);
  if (sync_metrics_lock.owns_lock()) {
    output.sync_completed = impl_->sync_completed;
    output.sync_failures = impl_->sync_failures;
    output.sync_avg_us = impl_->sync_duration.sample_count == 0
                             ? 0.0
                             : static_cast<double>(impl_->sync_duration.total_ns) /
                                   static_cast<double>(impl_->sync_duration.sample_count) /
                                   kNanosecondsPerMicrosecond;
    output.sync_max_us =
        static_cast<double>(impl_->sync_duration.max_ns) / kNanosecondsPerMicrosecond;
    impl_->sync_completed = 0;
    impl_->sync_failures = 0;
    impl_->sync_duration.Reset();
  }
  return output;
}

void SimViewer::RequestSyncFailureForTesting() noexcept {
  impl_->fail_next_sync.store(true, std::memory_order_release);
}

void SimViewer::HoldMailboxForTesting(std::chrono::milliseconds duration,
                                      std::atomic<bool>* lock_acquired) noexcept {
  std::lock_guard lock(impl_->mailbox_mutex);
  if (lock_acquired != nullptr) {
    lock_acquired->store(true, std::memory_order_release);
  }
  std::this_thread::sleep_for(duration);
}

void SimViewer::HoldSyncMetricsForTesting(std::chrono::milliseconds duration,
                                          std::atomic<bool>* lock_acquired) noexcept {
  std::lock_guard lock(impl_->sync_metrics_mutex);
  if (lock_acquired != nullptr) {
    lock_acquired->store(true, std::memory_order_release);
  }
  std::this_thread::sleep_for(duration);
}

void SimViewer::SetSyncDelayForTesting(std::chrono::milliseconds duration) noexcept {
  impl_->sync_delay_ns.store(ToNanoseconds(duration), std::memory_order_release);
}

bool SimViewer::SetControlForTesting(int actuator_id, double value) noexcept {
  if (impl_->simulate == nullptr) return false;
  try {
    const mujoco::MutexLock viewer_lock(impl_->simulate->mtx);
    if (actuator_id < 0 || static_cast<std::size_t>(actuator_id) >= impl_->simulate->ctrl_.size()) {
      return false;
    }
    impl_->simulate->ctrl_[actuator_id] = static_cast<mjtNum>(value);
    return true;
  } catch (...) {
    return false;
  }
}

bool SimViewer::RequestClearControlsForTesting() noexcept {
  if (impl_->simulate == nullptr) return false;
  try {
    const mujoco::MutexLock viewer_lock(impl_->simulate->mtx);
    impl_->simulate->pending_.zero_ctrl = true;
    return true;
  } catch (...) {
    return false;
  }
}

void SimViewer::PublishInputForTesting(const ViewerInputBatch& input) noexcept {
  try {
    impl_->PublishInput(input.actuator_writes, input.wrench_frame ? &*input.wrench_frame : nullptr);
  } catch (...) {
  }
}

void SimViewer::HoldInputMailboxForTesting(std::chrono::milliseconds duration,
                                           std::atomic<bool>* lock_acquired) noexcept {
  try {
    std::lock_guard lock(impl_->input_mutex);
    if (lock_acquired != nullptr) lock_acquired->store(true, std::memory_order_release);
    std::this_thread::sleep_for(duration);
  } catch (...) {
  }
}

bool SimViewer::sync_active_for_testing() const noexcept {
  return impl_->sync_active.load(std::memory_order_acquire);
}

double SimViewer::PendingTimeForTesting() const {
  std::lock_guard lock(impl_->mailbox_mutex);
  if (!impl_->pending_buffer.has_value()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return impl_->production_snapshots[*impl_->pending_buffer]->data()->time;
}

double SimViewer::DisplayTimeForTesting() const {
  if (impl_->simulate == nullptr) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const mujoco::MutexLock viewer_lock(impl_->simulate->mtx);
  return impl_->display_snapshot == nullptr ? std::numeric_limits<double>::quiet_NaN()
                                            : impl_->display_snapshot->data()->time;
}

bool SimViewer::snapshot_models_isolated_for_testing() const noexcept {
  return impl_->SnapshotStorageValid();
}

}  // namespace mujoco_sim::server
