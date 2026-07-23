#include <gtest/gtest.h>

#include <atomic>
#include <latch>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ecal_runtime.hpp"

namespace mujoco_sim::adapter_sdk::internal {
namespace {

TEST(EcalRuntimeConcurrencyTest, SharesConcurrentLeasesAndReinitializesAfterRelease) {
  constexpr std::size_t kThreadCount = 8;
  std::latch acquired(kThreadCount);
  std::latch release(1);
  std::atomic<std::size_t> failures = 0;
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);

  for (std::size_t index = 0; index < kThreadCount; ++index) {
    threads.emplace_back([&, index] {
      std::string error;
      auto lease = AcquireEcalRuntime("mujoco-sim-ecal-runtime-" + std::to_string(index), &error);
      if (!lease) ++failures;
      acquired.count_down();
      release.wait();
    });
  }
  acquired.wait();
  EXPECT_EQ(failures.load(), 0U);
  release.count_down();
  for (auto& thread : threads)
    thread.join();

  std::string error;
  auto lease = AcquireEcalRuntime("mujoco-sim-ecal-runtime-restart", &error);
  EXPECT_TRUE(lease) << error;
}

}  // namespace
}  // namespace mujoco_sim::adapter_sdk::internal
