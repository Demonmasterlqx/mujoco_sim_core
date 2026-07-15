#include <atomic>
#include <array>
#include <cstdlib>
#include <iostream>
#include <new>
#include <string>

#include <mujoco_sim/protocol/data_frame.hpp>

namespace {
std::atomic<std::uint64_t> allocations{0};
}

void* operator new(std::size_t size) {
  allocations.fetch_add(1, std::memory_order_relaxed);
  if (void* memory = std::malloc(size)) return memory;
  throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
  allocations.fetch_add(1, std::memory_order_relaxed);
  if (void* memory = std::malloc(size)) return memory;
  throw std::bad_alloc();
}

void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept {
  std::free(memory);
}

int main() {
  mujoco_sim::protocol::DataFrameHeader header;
  header.kind = mujoco_sim::protocol::DataFrameKind::kTick;
  header.session_id = 1;
  header.epoch = 2;
  header.tick_id = 3;
  header.layout = {4, 5};
  std::array<double, 32> scalars{};
  std::array<double, 32> decoded{};
  std::string encoded;
  encoded.reserve(mujoco_sim::protocol::kDataFrameHeaderSize +
                  scalars.size() * sizeof(double));
  mujoco_sim::protocol::DataFrameError error;
  if (!mujoco_sim::protocol::EncodeDataFrame(header, scalars, &encoded,
                                              &error)) {
    return 1;
  }

  const auto before = allocations.load(std::memory_order_relaxed);
  for (std::size_t iteration = 0; iteration < 10000; ++iteration) {
    header.tick_id = iteration + 1;
    mujoco_sim::protocol::ParsedDataFrame parsed;
    if (!mujoco_sim::protocol::EncodeDataFrame(header, scalars, &encoded,
                                                &error) ||
        !mujoco_sim::protocol::ParseDataFrame(encoded, &parsed, &error) ||
        !mujoco_sim::protocol::DecodeDataFrameScalars(parsed, decoded,
                                                       &error)) {
      return 1;
    }
  }
  const auto steady_state_allocations =
      allocations.load(std::memory_order_relaxed) - before;
  if (steady_state_allocations != 0) {
    std::cerr << "steady-state codec allocations="
              << steady_state_allocations << '\n';
    return 1;
  }
  return 0;
}
