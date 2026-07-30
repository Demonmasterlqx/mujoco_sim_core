#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <new>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include <mujoco/mujoco.h>
#include <mujoco_sim/core/simulation_core.hpp>

#include "server_application_test_harness.hpp"
#include "sim_viewer.hpp"

namespace {

using namespace std::chrono_literals;

std::atomic<bool> track_allocations{false};
std::atomic<std::uint64_t> allocation_count{0};

struct ConfiguredViewer {
  explicit ConfiguredViewer(mujoco_sim::server::SimViewer::InputOptions options)
      : viewer(options) {}

  mujoco_sim::core::SimulationCore core;
  mujoco_sim::server::SimViewer viewer;
};

std::unique_ptr<ConfiguredViewer> CreateViewer(
    std::string* error, mujoco_sim::server::SimViewer::InputOptions options = {},
    const char* model_path = MUJOCO_SIM_SERVER_TEST_MODEL) {
  auto configured = std::make_unique<ConfiguredViewer>(options);
  const auto model_load = configured->core.LoadModel(model_path);
  if (!model_load.response.loaded) {
    *error = model_load.response.message;
    return nullptr;
  }
  if (!configured->viewer.Initialize(error)) {
    return nullptr;
  }
  auto display = configured->core.CreateSnapshot(error);
  if (display == nullptr) {
    return nullptr;
  }
  auto first = configured->core.CreateSnapshot(error);
  if (first == nullptr) {
    return nullptr;
  }
  auto second = configured->core.CloneSnapshot(*first, error);
  if (first == nullptr || second == nullptr ||
      !configured->viewer.QueueLoad(std::move(display), std::move(first), std::move(second),
                                    model_path, error)) {
    return nullptr;
  }
  return configured;
}

bool CaptureAndSubmit(ConfiguredViewer* configured, std::string* error) {
  auto lease = configured->viewer.TryAcquireSnapshot();
  return lease && configured->core.UpdateSnapshot(lease.snapshot(), error) &&
         configured->viewer.TrySubmitSnapshot(std::move(lease));
}

bool CaptureTimeAndSubmit(ConfiguredViewer* configured, double time, std::string* error) {
  auto lease = configured->viewer.TryAcquireSnapshot();
  if (!lease || !configured->core.UpdateSnapshot(lease.snapshot(), error)) {
    return false;
  }
  lease.snapshot().data()->time = time;
  return configured->viewer.TrySubmitSnapshot(std::move(lease));
}

class TemporaryLog {
 public:
  TemporaryLog() {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("mujoco_sim_viewer_failure_" + std::to_string(suffix) + ".log");
  }

  ~TemporaryLog() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  const std::filesystem::path& path() const noexcept { return path_; }

  std::string Read() const {
    std::ifstream stream(path_);
    std::ostringstream contents;
    contents << stream.rdbuf();
    return contents.str();
  }

 private:
  std::filesystem::path path_;
};

template <typename Predicate>
bool WaitUntil(Predicate predicate, std::chrono::steady_clock::duration timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return predicate();
    }
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

}  // namespace

void* operator new(std::size_t size) {
  if (void* pointer = std::malloc(size == 0 ? 1 : size)) {
    if (track_allocations.load(std::memory_order_relaxed)) {
      allocation_count.fetch_add(1, std::memory_order_relaxed);
    }
    return pointer;
  }
  throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
  return ::operator new(size);
}

void operator delete(void* pointer) noexcept {
  std::free(pointer);
}

void operator delete[](void* pointer) noexcept {
  std::free(pointer);
}

void operator delete(void* pointer, std::size_t) noexcept {
  std::free(pointer);
}

void operator delete[](void* pointer, std::size_t) noexcept {
  std::free(pointer);
}

namespace {

TEST(SimViewerTest, DepthOneMailboxReplacesPendingAndBoundsBuffers) {
  std::string error;
  auto configured = CreateViewer(&error);
  ASSERT_NE(configured, nullptr) << error;

  ASSERT_TRUE(configured->viewer.snapshot_models_isolated_for_testing());
  ASSERT_TRUE(CaptureTimeAndSubmit(configured.get(), 1.0, &error)) << error;
  EXPECT_DOUBLE_EQ(configured->viewer.PendingTimeForTesting(), 1.0);
  ASSERT_TRUE(CaptureTimeAndSubmit(configured.get(), 2.0, &error)) << error;
  EXPECT_DOUBLE_EQ(configured->viewer.PendingTimeForTesting(), 2.0);
  auto held = configured->viewer.TryAcquireSnapshot();
  ASSERT_TRUE(held);
  auto exhausted = configured->viewer.TryAcquireSnapshot();
  EXPECT_FALSE(exhausted);
  held = {};

  const auto metrics = configured->viewer.GetAndResetIntervalMetrics();
  EXPECT_EQ(metrics.capture_attempts, 4U);
  EXPECT_EQ(metrics.capture_submitted, 2U);
  EXPECT_EQ(metrics.capture_skipped, 2U);
  EXPECT_EQ(metrics.pending_replaced, 1U);
  EXPECT_EQ(metrics.sync_completed, 0U);
  EXPECT_EQ(metrics.sync_failures, 0U);
  EXPECT_GE(metrics.capture_max_us, metrics.capture_avg_us);

  configured->viewer.RequestClose();
  EXPECT_TRUE(configured->viewer.RenderLoop(&error)) << error;
}

TEST(SimViewerTest, ExtractsOnlyFiniteDynamicBodyWrenchesTransactionally) {
  std::string error;
  auto configured =
      CreateViewer(&error, {.mouse_force_enabled = true}, MUJOCO_SIM_SERVER_DUAL_TEST_MODEL);
  ASSERT_NE(configured, nullptr) << error;
  auto snapshot = configured->core.CreateSnapshot(&error);
  ASSERT_NE(snapshot, nullptr) << error;
  const int dynamic_body = mj_name2id(snapshot->model(), mjOBJ_BODY, "first_link");
  const int static_body = mj_name2id(snapshot->model(), mjOBJ_BODY, "static_link");
  ASSERT_GT(dynamic_body, 0);
  ASSERT_GT(static_body, 0);
  snapshot->data()->xfrc_applied[6 * dynamic_body] = 1.0;
  snapshot->data()->xfrc_applied[6 * dynamic_body + 5] = 2.0;
  snapshot->data()->xfrc_applied[6 * static_body] = 9.0;

  std::vector<mujoco_sim::core::InteractiveBodyWrench> output;
  ASSERT_TRUE(mujoco_sim::server::detail::ExtractViewerBodyWrenches(snapshot->model(),
                                                                    snapshot->data(), &output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front().body_id, dynamic_body);
  EXPECT_DOUBLE_EQ(output.front().force[0], 1.0);
  EXPECT_DOUBLE_EQ(output.front().torque[2], 2.0);

  snapshot->data()->xfrc_applied[6 * dynamic_body] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mujoco_sim::server::detail::ExtractViewerBodyWrenches(snapshot->model(),
                                                                     snapshot->data(), &output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front().body_id, dynamic_body);
}

TEST(SimViewerTest, InputMailboxMergesControlAndKeepsExplicitEmptyWrench) {
  std::string error;
  auto configured = CreateViewer(&error, {.control_enabled = true, .mouse_force_enabled = true});
  ASSERT_NE(configured, nullptr) << error;

  mujoco_sim::server::SimViewer::ViewerInputBatch first;
  first.actuator_writes = {{.actuator_id = 0, .value = 0.2}};
  first.wrench_frame.emplace();
  first.wrench_frame->sequence = 1;
  first.wrench_frame->body_wrenches = {
      {.body_id = 1, .force = {1.0, 0.0, 0.0}},
  };
  configured->viewer.PublishInputForTesting(first);

  mujoco_sim::server::SimViewer::ViewerInputBatch second;
  second.actuator_writes = {{.actuator_id = 0, .value = 0.4}};
  second.wrench_frame.emplace();
  second.wrench_frame->sequence = 2;
  configured->viewer.PublishInputForTesting(second);

  mujoco_sim::server::SimViewer::ViewerInputBatch consumed;
  ASSERT_TRUE(configured->viewer.TryConsumeInput(&consumed));
  ASSERT_EQ(consumed.actuator_writes.size(), 1U);
  EXPECT_DOUBLE_EQ(consumed.actuator_writes.front().value, 0.4);
  ASSERT_TRUE(consumed.wrench_frame.has_value());
  EXPECT_EQ(consumed.wrench_frame->sequence, 2U);
  EXPECT_TRUE(consumed.wrench_frame->body_wrenches.empty());
  EXPECT_FALSE(configured->viewer.TryConsumeInput(&consumed));
}

TEST(SimViewerTest, DisabledInputOptionsRejectSyntheticExtractedValues) {
  std::string error;
  auto configured = CreateViewer(&error);
  ASSERT_NE(configured, nullptr) << error;
  mujoco_sim::server::SimViewer::ViewerInputBatch input;
  input.actuator_writes = {{.actuator_id = 0, .value = 0.5}};
  input.wrench_frame.emplace();
  input.wrench_frame->sequence = 1;
  input.wrench_frame->body_wrenches = {
      {.body_id = 1, .force = {1.0, 0.0, 0.0}},
  };
  configured->viewer.PublishInputForTesting(input);
  EXPECT_FALSE(configured->viewer.TryConsumeInput(&input));
}

TEST(SimViewerTest, InputConsumerDoesNotWaitForMailboxOwner) {
  std::string error;
  auto configured = CreateViewer(&error, {.control_enabled = true});
  ASSERT_NE(configured, nullptr) << error;
  mujoco_sim::server::SimViewer::ViewerInputBatch input;
  input.actuator_writes = {{.actuator_id = 0, .value = 0.5}};
  configured->viewer.PublishInputForTesting(input);

  std::atomic<bool> locked{false};
  std::jthread holder([&] { configured->viewer.HoldInputMailboxForTesting(500ms, &locked); });
  ASSERT_TRUE(WaitUntil([&] { return locked.load(std::memory_order_acquire); }, 1s));
  mujoco_sim::server::SimViewer::ViewerInputBatch output;
  const auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(configured->viewer.TryConsumeInput(&output));
  EXPECT_LT(std::chrono::steady_clock::now() - started, 100ms);
  holder.join();
  EXPECT_TRUE(configured->viewer.TryConsumeInput(&output));
}

TEST(SimViewerTest, EnabledIdleAndStableWrenchReusePreallocatedInputStorage) {
  std::string error;
  auto configured = CreateViewer(&error, {.control_enabled = true, .mouse_force_enabled = true},
                                 MUJOCO_SIM_SERVER_DUAL_TEST_MODEL);
  ASSERT_NE(configured, nullptr) << error;

  mujoco_sim::server::SimViewer::ViewerInputBatch input;
  input.wrench_frame.emplace();
  input.wrench_frame->body_wrenches = {
      {.body_id = 1, .force = {1.0, 0.0, 0.0}},
  };
  mujoco_sim::server::SimViewer::ViewerInputBatch output;
  configured->viewer.PublishInputForTesting(input);
  ASSERT_TRUE(configured->viewer.TryConsumeInput(&output));

  allocation_count.store(0, std::memory_order_relaxed);
  track_allocations.store(true, std::memory_order_release);
  bool all_consumed = true;
  for (std::uint64_t sequence = 1; sequence <= 1000; ++sequence) {
    input.wrench_frame->sequence = sequence;
    configured->viewer.PublishInputForTesting(input);
    all_consumed = configured->viewer.TryConsumeInput(&output) && all_consumed;
  }
  const bool idle_has_input = configured->viewer.TryConsumeInput(&output);
  track_allocations.store(false, std::memory_order_release);

  EXPECT_TRUE(all_consumed);
  EXPECT_FALSE(idle_has_input);
  EXPECT_EQ(allocation_count.load(std::memory_order_relaxed), 0U);
}

TEST(SimViewerTest, ValidatesEverySnapshotSignatureAndBufferSize) {
  const int display_model = 1;
  const int producer_model = 2;
  const int display_data = 3;
  const int first_data = 4;
  const int second_data = 5;
  const mujoco_sim::server::detail::ViewerSnapshotMetadata display{
      .model_identity = &display_model,
      .data_identity = &display_data,
      .model_signature = 17,
      .data_signature = 17,
      .model_buffer_bytes = 100,
      .data_buffer_bytes = 200,
  };
  const mujoco_sim::server::detail::ViewerSnapshotMetadata first{
      .model_identity = &producer_model,
      .data_identity = &first_data,
      .model_signature = 17,
      .data_signature = 17,
      .model_buffer_bytes = 100,
      .data_buffer_bytes = 200,
  };
  const mujoco_sim::server::detail::ViewerSnapshotMetadata second{
      .model_identity = &producer_model,
      .data_identity = &second_data,
      .model_signature = 17,
      .data_signature = 17,
      .model_buffer_bytes = 100,
      .data_buffer_bytes = 200,
  };
  ASSERT_TRUE(mujoco_sim::server::detail::ViewerSnapshotsCompatible(display, first, second));

  auto mismatch = first;
  mismatch.model_signature = 18;
  EXPECT_FALSE(mujoco_sim::server::detail::ViewerSnapshotsCompatible(display, mismatch, second));

  mismatch = display;
  mismatch.data_signature = 18;
  EXPECT_FALSE(mujoco_sim::server::detail::ViewerSnapshotsCompatible(mismatch, first, second));
  mismatch = first;
  mismatch.data_signature = 18;
  EXPECT_FALSE(mujoco_sim::server::detail::ViewerSnapshotsCompatible(display, mismatch, second));
  mismatch = second;
  mismatch.data_signature = 18;
  EXPECT_FALSE(mujoco_sim::server::detail::ViewerSnapshotsCompatible(display, first, mismatch));

  mismatch = second;
  mismatch.model_buffer_bytes = 101;
  EXPECT_FALSE(mujoco_sim::server::detail::ViewerSnapshotsCompatible(display, first, mismatch));
  mismatch = second;
  mismatch.data_buffer_bytes = 201;
  EXPECT_FALSE(mujoco_sim::server::detail::ViewerSnapshotsCompatible(display, first, mismatch));
}

TEST(SimViewerTest, BurstProductionReusesFixedBuffersWithoutAllocating) {
  std::string error;
  auto configured = CreateViewer(&error);
  ASSERT_NE(configured, nullptr) << error;
  ASSERT_TRUE(CaptureAndSubmit(configured.get(), &error)) << error;
  configured->viewer.GetAndResetIntervalMetrics();

  allocation_count.store(0, std::memory_order_relaxed);
  track_allocations.store(true, std::memory_order_release);
  bool all_submitted = true;
  for (int frame = 0; frame < 1000; ++frame) {
    all_submitted = CaptureAndSubmit(configured.get(), &error) && all_submitted;
  }
  track_allocations.store(false, std::memory_order_release);

  ASSERT_TRUE(all_submitted) << error;
  EXPECT_EQ(allocation_count.load(std::memory_order_relaxed), 0U);
  const auto metrics = configured->viewer.GetAndResetIntervalMetrics();
  EXPECT_EQ(metrics.capture_attempts, 1000U);
  EXPECT_EQ(metrics.capture_submitted, 1000U);
  EXPECT_EQ(metrics.capture_skipped, 0U);
  EXPECT_EQ(metrics.pending_replaced, 1000U);

  configured->viewer.RequestClose();
  EXPECT_TRUE(configured->viewer.RenderLoop(&error)) << error;
}

TEST(SimViewerTest, LoadsSynchronizesSubmittedDataAndClosesWindow) {
  std::string error;
  auto configured = CreateViewer(&error);
  ASSERT_NE(configured, nullptr) << error;
  std::atomic<bool> loaded{false};
  std::atomic<bool> latest_displayed{false};
  std::string producer_error;
  std::thread producer([&] {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!configured->viewer.model_loaded() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(2ms);
    }
    loaded.store(configured->viewer.model_loaded(), std::memory_order_release);
    if (loaded.load(std::memory_order_acquire)) {
      CaptureTimeAndSubmit(configured.get(), 42.0, &producer_error);
      latest_displayed.store(
          WaitUntil([&] { return configured->viewer.DisplayTimeForTesting() == 42.0; }, 2s),
          std::memory_order_release);
    }
    configured->viewer.RequestClose();
  });

  EXPECT_TRUE(configured->viewer.RenderLoop(&error)) << error;
  producer.join();
  EXPECT_TRUE(loaded.load(std::memory_order_acquire)) << producer_error;
  EXPECT_TRUE(latest_displayed.load(std::memory_order_acquire)) << producer_error;
  const auto metrics = configured->viewer.GetAndResetIntervalMetrics();
  EXPECT_GT(metrics.capture_submitted, 0U);
  EXPECT_GT(metrics.sync_completed, 0U);
  EXPECT_EQ(metrics.sync_failures, 0U);
  EXPECT_TRUE(configured->viewer.close_requested());
}

TEST(SimViewerTest, ExtractsPassiveControlEditAndClearAllThroughRealSync) {
  std::string error;
  auto configured = CreateViewer(&error, {.control_enabled = true});
  ASSERT_NE(configured, nullptr) << error;

  std::atomic<bool> succeeded{false};
  std::string producer_error;
  std::jthread producer([&] {
    const auto fail = [&] {
      configured->viewer.RequestClose();
      return;
    };
    if (!WaitUntil([&] { return configured->viewer.model_loaded(); }, 2s) ||
        !configured->viewer.SetControlForTesting(0, 0.5) ||
        !CaptureAndSubmit(configured.get(), &producer_error)) {
      fail();
      return;
    }
    mujoco_sim::server::SimViewer::ViewerInputBatch input;
    if (!WaitUntil([&] { return configured->viewer.TryConsumeInput(&input); }, 2s) ||
        input.actuator_writes.size() != 1 || input.actuator_writes.front().value != 0.5) {
      producer_error = "viewer did not extract the injected Control edit";
      fail();
      return;
    }
    if (!configured->core.Step({.actuator_writes = input.actuator_writes}).step_completed ||
        !configured->viewer.RequestClearControlsForTesting() ||
        !CaptureAndSubmit(configured.get(), &producer_error)) {
      fail();
      return;
    }
    input.actuator_writes.clear();
    if (!WaitUntil([&] { return configured->viewer.TryConsumeInput(&input); }, 2s) ||
        input.actuator_writes.size() != 1 || input.actuator_writes.front().value != 0.0) {
      producer_error = "viewer did not extract Clear all";
      fail();
      return;
    }
    succeeded.store(true, std::memory_order_release);
    configured->viewer.RequestClose();
  });

  EXPECT_TRUE(configured->viewer.RenderLoop(&error)) << error;
  producer.join();
  EXPECT_TRUE(succeeded.load(std::memory_order_acquire)) << producer_error;
}

TEST(SimViewerTest, ProducerCallsRemainBoundedWhileWorkerConsumes) {
  std::string error;
  auto configured = CreateViewer(&error);
  ASSERT_NE(configured, nullptr) << error;
  std::chrono::steady_clock::duration maximum_call{};
  std::string producer_error;
  std::thread producer([&] {
    for (int frame = 0; frame < 2000; ++frame) {
      const auto started = std::chrono::steady_clock::now();
      auto lease = configured->viewer.TryAcquireSnapshot();
      if (lease && configured->core.UpdateSnapshot(lease.snapshot(), &producer_error)) {
        configured->viewer.TrySubmitSnapshot(std::move(lease));
      }
      maximum_call = std::max(maximum_call, std::chrono::steady_clock::now() - started);
    }
    configured->viewer.RequestClose();
  });

  EXPECT_TRUE(configured->viewer.RenderLoop(&error)) << error;
  producer.join();
  EXPECT_LT(maximum_call, 50ms) << producer_error;
  const auto metrics = configured->viewer.GetAndResetIntervalMetrics();
  EXPECT_EQ(metrics.capture_attempts, 2000U);
  EXPECT_EQ(metrics.capture_attempts, metrics.capture_submitted + metrics.capture_skipped);
}

TEST(SimViewerTest, ProducerTryLockCallsDoNotWaitForMailboxOwner) {
  std::string error;
  auto configured = CreateViewer(&error);
  ASSERT_NE(configured, nullptr) << error;

  auto lease = configured->viewer.TryAcquireSnapshot();
  ASSERT_TRUE(lease);
  ASSERT_TRUE(configured->core.UpdateSnapshot(lease.snapshot(), &error)) << error;

  std::atomic<bool> mailbox_locked{false};
  std::jthread lock_holder(
      [&] { configured->viewer.HoldMailboxForTesting(500ms, &mailbox_locked); });
  ASSERT_TRUE(WaitUntil([&] { return mailbox_locked.load(std::memory_order_acquire); }, 1s));

  const auto acquire_started = std::chrono::steady_clock::now();
  auto unavailable = configured->viewer.TryAcquireSnapshot();
  const auto acquire_elapsed = std::chrono::steady_clock::now() - acquire_started;
  EXPECT_FALSE(unavailable);
  EXPECT_LT(acquire_elapsed, 100ms);

  const auto submit_started = std::chrono::steady_clock::now();
  const bool submitted = configured->viewer.TrySubmitSnapshot(std::move(lease));
  const auto submit_elapsed = std::chrono::steady_clock::now() - submit_started;
  EXPECT_FALSE(submitted);
  EXPECT_LT(submit_elapsed, 100ms);

  const auto close_started = std::chrono::steady_clock::now();
  configured->viewer.RequestClose();
  const auto close_elapsed = std::chrono::steady_clock::now() - close_started;
  EXPECT_LT(close_elapsed, 100ms);

  lock_holder.join();
  const auto metrics = configured->viewer.GetAndResetIntervalMetrics();
  EXPECT_EQ(metrics.capture_attempts, 2U);
  EXPECT_EQ(metrics.capture_submitted, 0U);
  EXPECT_EQ(metrics.capture_skipped, 2U);

  EXPECT_TRUE(configured->viewer.RenderLoop(&error)) << error;
}

TEST(SimViewerTest, SyncMetricsRemainCoherentWhenSummaryTryLockContends) {
  std::string error;
  auto configured = CreateViewer(&error);
  ASSERT_NE(configured, nullptr) << error;

  mujoco_sim::server::SimViewer::IntervalMetrics contended_metrics;
  mujoco_sim::server::SimViewer::IntervalMetrics retained_metrics;
  std::string producer_error;
  std::jthread producer([&] {
    if (!WaitUntil([&] { return configured->viewer.model_loaded(); }, 2s) ||
        !CaptureTimeAndSubmit(configured.get(), 7.0, &producer_error) ||
        !WaitUntil([&] { return configured->viewer.DisplayTimeForTesting() == 7.0; }, 2s)) {
      configured->viewer.RequestClose();
      return;
    }

    std::atomic<bool> metrics_locked{false};
    std::jthread lock_holder(
        [&] { configured->viewer.HoldSyncMetricsForTesting(500ms, &metrics_locked); });
    if (!WaitUntil([&] { return metrics_locked.load(std::memory_order_acquire); }, 1s)) {
      configured->viewer.RequestClose();
      return;
    }
    const auto started = std::chrono::steady_clock::now();
    contended_metrics = configured->viewer.GetAndResetIntervalMetrics();
    EXPECT_LT(std::chrono::steady_clock::now() - started, 100ms);
    lock_holder.join();
    retained_metrics = configured->viewer.GetAndResetIntervalMetrics();
    configured->viewer.RequestClose();
  });

  EXPECT_TRUE(configured->viewer.RenderLoop(&error)) << error;
  producer.join();
  EXPECT_TRUE(producer_error.empty()) << producer_error;
  EXPECT_EQ(contended_metrics.sync_completed, 0U);
  EXPECT_EQ(contended_metrics.sync_failures, 0U);
  EXPECT_DOUBLE_EQ(contended_metrics.sync_avg_us, 0.0);
  EXPECT_DOUBLE_EQ(contended_metrics.sync_max_us, 0.0);
  EXPECT_EQ(retained_metrics.sync_completed, 1U);
  EXPECT_EQ(retained_metrics.sync_failures, 0U);
  EXPECT_GT(retained_metrics.sync_avg_us, 0.0);
  EXPECT_GE(retained_metrics.sync_max_us, retained_metrics.sync_avg_us);
}

TEST(SimViewerTest, WrongViewerSubmissionReturnsAndInvalidatesLease) {
  std::string error;
  auto first = CreateViewer(&error);
  ASSERT_NE(first, nullptr) << error;
  auto second = CreateViewer(&error);
  ASSERT_NE(second, nullptr) << error;

  auto lease = first->viewer.TryAcquireSnapshot();
  ASSERT_TRUE(lease);
  ASSERT_TRUE(first->core.UpdateSnapshot(lease.snapshot(), &error)) << error;
  EXPECT_FALSE(second->viewer.TrySubmitSnapshot(std::move(lease)));
  EXPECT_FALSE(lease);

  const auto first_metrics = first->viewer.GetAndResetIntervalMetrics();
  EXPECT_EQ(first_metrics.capture_attempts, 1U);
  EXPECT_EQ(first_metrics.capture_submitted, 0U);
  EXPECT_EQ(first_metrics.capture_skipped, 1U);

  first->viewer.RequestClose();
  second->viewer.RequestClose();
  EXPECT_TRUE(first->viewer.RenderLoop(&error)) << error;
  EXPECT_TRUE(second->viewer.RenderLoop(&error)) << error;
}

TEST(SimViewerTest, BufferExhaustionAndCloseRemainBoundedDuringActiveSync) {
  std::string error;
  auto configured = CreateViewer(&error);
  ASSERT_NE(configured, nullptr) << error;
  configured->viewer.SetSyncDelayForTesting(1s);

  std::atomic<bool> loaded{false};
  std::atomic<bool> first_submitted{false};
  std::atomic<bool> sync_active{false};
  std::atomic<bool> second_submitted{false};
  std::atomic<bool> third_skipped{false};
  std::chrono::steady_clock::duration third_acquire_elapsed{};
  std::chrono::steady_clock::duration close_elapsed{};
  std::string producer_error;
  std::jthread producer([&] {
    loaded.store(WaitUntil([&] { return configured->viewer.model_loaded(); }, 2s),
                 std::memory_order_release);
    if (!loaded.load(std::memory_order_acquire)) {
      configured->viewer.RequestClose();
      return;
    }

    first_submitted.store(CaptureAndSubmit(configured.get(), &producer_error),
                          std::memory_order_release);
    sync_active.store(WaitUntil([&] { return configured->viewer.sync_active_for_testing(); }, 2s),
                      std::memory_order_release);
    if (!sync_active.load(std::memory_order_acquire)) {
      configured->viewer.RequestClose();
      return;
    }

    second_submitted.store(CaptureAndSubmit(configured.get(), &producer_error),
                           std::memory_order_release);
    const auto acquire_started = std::chrono::steady_clock::now();
    auto third = configured->viewer.TryAcquireSnapshot();
    third_acquire_elapsed = std::chrono::steady_clock::now() - acquire_started;
    third_skipped.store(!third, std::memory_order_release);

    const auto close_started = std::chrono::steady_clock::now();
    configured->viewer.RequestClose();
    close_elapsed = std::chrono::steady_clock::now() - close_started;
  });

  const auto render_started = std::chrono::steady_clock::now();
  EXPECT_TRUE(configured->viewer.RenderLoop(&error)) << error;
  const auto render_elapsed = std::chrono::steady_clock::now() - render_started;
  producer.join();

  EXPECT_TRUE(loaded.load(std::memory_order_acquire)) << producer_error;
  EXPECT_TRUE(first_submitted.load(std::memory_order_acquire)) << producer_error;
  EXPECT_TRUE(sync_active.load(std::memory_order_acquire));
  EXPECT_TRUE(second_submitted.load(std::memory_order_acquire)) << producer_error;
  EXPECT_TRUE(third_skipped.load(std::memory_order_acquire));
  EXPECT_LT(third_acquire_elapsed, 100ms);
  EXPECT_LT(close_elapsed, 100ms);
  EXPECT_LT(render_elapsed, 3s);

  const auto metrics = configured->viewer.GetAndResetIntervalMetrics();
  EXPECT_EQ(metrics.capture_attempts, 3U);
  EXPECT_EQ(metrics.capture_submitted, 2U);
  EXPECT_EQ(metrics.capture_skipped, 1U);
  EXPECT_EQ(metrics.pending_replaced, 0U);
  EXPECT_EQ(metrics.sync_completed, 0U);
  EXPECT_EQ(metrics.sync_failures, 0U);
}

TEST(SimViewerTest, WorkerFailureIsStickyAndContained) {
  std::string error;
  auto configured = CreateViewer(&error);
  ASSERT_NE(configured, nullptr) << error;
  configured->viewer.RequestSyncFailureForTesting();
  ASSERT_TRUE(CaptureAndSubmit(configured.get(), &error)) << error;

  EXPECT_FALSE(configured->viewer.RenderLoop(&error));
  EXPECT_NE(error.find("injected viewer sync worker failure"), std::string::npos);
  const auto metrics = configured->viewer.GetAndResetIntervalMetrics();
  EXPECT_EQ(metrics.sync_failures, 1U);
  EXPECT_TRUE(configured->viewer.close_requested());
}

TEST(SimViewerTest, ClosesWithPendingModelLoad) {
  std::string error;
  auto configured = CreateViewer(&error);
  ASSERT_NE(configured, nullptr) << error;
  ASSERT_TRUE(CaptureAndSubmit(configured.get(), &error)) << error;
  configured->viewer.RequestClose();
  EXPECT_TRUE(configured->viewer.RenderLoop(&error)) << error;
  EXPECT_TRUE(configured->viewer.close_requested());
}

TEST(SimViewerTest, WorkerFailurePropagatesThroughServerApplication) {
  TemporaryLog log;
  mujoco_sim::server::ServerConfig config;
  config.server_name = "viewer_sync_failure_test";
  config.model_path = MUJOCO_SIM_SERVER_TEST_MODEL;
  config.headless = false;
  config.viewer_sync_period_ms = 1;
  config.log_file = log.path().string();
  config.log_summary_period_ms = 0;

  mujoco_sim::server::ServerApplicationTestHarness application(std::move(config));
  std::string error;
  ASSERT_TRUE(application.Start(&error)) << error;
  ASSERT_TRUE(application.RequestViewerSyncFailure());
  EXPECT_EQ(application.Run(), 2);

  const auto contents = log.Read();
  EXPECT_NE(contents.find("event=server_stopped"), std::string::npos);
  EXPECT_NE(contents.find("reason=\"viewer_sync_failed\""), std::string::npos);
  EXPECT_NE(contents.find("exit_code=2"), std::string::npos);
  EXPECT_EQ(contents.find("reason=\"viewer_closed\""), std::string::npos);
}

}  // namespace
