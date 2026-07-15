#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <mujoco_sim/protocol/data_frame.hpp>

namespace {

using Clock = std::chrono::steady_clock;

template <typename Action>
double MeasureP95(Action action) {
  std::vector<double> samples;
  samples.reserve(10000);
  for (std::size_t index = 0; index < 10000; ++index) {
    const auto start = Clock::now();
    if (!action()) return -1.0;
    samples.push_back(
        std::chrono::duration<double, std::micro>(Clock::now() - start)
            .count());
  }
  std::sort(samples.begin(), samples.end());
  return samples[static_cast<std::size_t>(0.95 * (samples.size() - 1))];
}

}  // namespace

int main() {
  mujoco_sim::protocol::DataFrameHeader tick_header;
  tick_header.kind = mujoco_sim::protocol::DataFrameKind::kTick;
  tick_header.session_id = 1;
  tick_header.epoch = 2;
  tick_header.tick_id = 3;
  tick_header.layout = {4, 5};
  auto commit_header = tick_header;
  commit_header.kind = mujoco_sim::protocol::DataFrameKind::kCommit;
  std::array<double, 32> observations{};
  std::array<double, 8> commands{};
  std::array<double, 32> decoded_observations{};
  std::array<double, 8> decoded_commands{};
  std::string tick_bytes, commit_bytes;
  tick_bytes.reserve(336);
  commit_bytes.reserve(144);
  mujoco_sim::protocol::DataFrameError error;

  const double tick_encode_p95 = MeasureP95([&] {
    return mujoco_sim::protocol::EncodeDataFrame(
        tick_header, observations, &tick_bytes, &error);
  });
  const double tick_decode_p95 = MeasureP95([&] {
    mujoco_sim::protocol::ParsedDataFrame parsed;
    return mujoco_sim::protocol::ParseDataFrame(tick_bytes, &parsed, &error) &&
           mujoco_sim::protocol::DecodeDataFrameScalars(
               parsed, decoded_observations, &error);
  });
  const double commit_encode_p95 = MeasureP95([&] {
    return mujoco_sim::protocol::EncodeDataFrame(
        commit_header, commands, &commit_bytes, &error);
  });
  const double commit_decode_p95 = MeasureP95([&] {
    mujoco_sim::protocol::ParsedDataFrame parsed;
    return mujoco_sim::protocol::ParseDataFrame(commit_bytes, &parsed,
                                                 &error) &&
           mujoco_sim::protocol::DecodeDataFrameScalars(
               parsed, decoded_commands, &error);
  });

  std::cout << "{\"iterations\":10000,\"tick_scalars\":32,"
            << "\"command_scalars\":8,\"tick_bytes\":"
            << tick_bytes.size() << ",\"commit_bytes\":"
            << commit_bytes.size() << ",\"tick_encode_p95_us\":"
            << tick_encode_p95 << ",\"tick_decode_p95_us\":"
            << tick_decode_p95 << ",\"commit_encode_p95_us\":"
            << commit_encode_p95 << ",\"commit_decode_p95_us\":"
            << commit_decode_p95 << "}\n";
  return tick_encode_p95 >= 0.0 && tick_encode_p95 < 50.0 &&
                 tick_decode_p95 >= 0.0 && tick_decode_p95 < 30.0 &&
                 commit_encode_p95 >= 0.0 && commit_encode_p95 < 30.0 &&
                 commit_decode_p95 >= 0.0 && commit_decode_p95 < 30.0
             ? 0
             : 1;
}
