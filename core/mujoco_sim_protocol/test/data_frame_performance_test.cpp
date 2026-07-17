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
    samples.push_back(std::chrono::duration<double, std::micro>(Clock::now() - start).count());
  }
  std::sort(samples.begin(), samples.end());
  return samples[static_cast<std::size_t>(0.95 * (samples.size() - 1))];
}

}  // namespace

int main() {
  mujoco_sim::protocol::DataFrameHeader state_header;
  state_header.kind = mujoco_sim::protocol::DataFrameKind::kControllerState;
  state_header.session_id = 1;
  state_header.epoch = 2;
  state_header.tick_id = 3;
  state_header.layout = {4, 5};
  state_header.sequence = 1;
  auto command_header = state_header;
  command_header.kind = mujoco_sim::protocol::DataFrameKind::kControllerCommand;
  std::array<double, 32> observations{};
  std::array<double, 8> commands{};
  std::array<double, 32> decoded_observations{};
  std::array<double, 8> decoded_commands{};
  std::string state_bytes, command_bytes;
  state_bytes.reserve(336);
  command_bytes.reserve(144);
  mujoco_sim::protocol::DataFrameError error;

  const double state_encode_p95 = MeasureP95([&] {
    return mujoco_sim::protocol::EncodeDataFrame(state_header, observations, &state_bytes, &error);
  });
  const double state_decode_p95 = MeasureP95([&] {
    mujoco_sim::protocol::ParsedDataFrame parsed;
    return mujoco_sim::protocol::ParseDataFrame(state_bytes, &parsed, &error) &&
           mujoco_sim::protocol::DecodeDataFrameScalars(parsed, decoded_observations, &error);
  });
  const double command_encode_p95 = MeasureP95([&] {
    return mujoco_sim::protocol::EncodeDataFrame(command_header, commands, &command_bytes, &error);
  });
  const double command_decode_p95 = MeasureP95([&] {
    mujoco_sim::protocol::ParsedDataFrame parsed;
    return mujoco_sim::protocol::ParseDataFrame(command_bytes, &parsed, &error) &&
           mujoco_sim::protocol::DecodeDataFrameScalars(parsed, decoded_commands, &error);
  });

  std::cout << "{\"iterations\":10000,\"state_scalars\":32,"
            << "\"command_scalars\":8,\"state_bytes\":" << state_bytes.size()
            << ",\"command_bytes\":" << command_bytes.size()
            << ",\"state_encode_p95_us\":" << state_encode_p95
            << ",\"state_decode_p95_us\":" << state_decode_p95
            << ",\"command_encode_p95_us\":" << command_encode_p95
            << ",\"command_decode_p95_us\":" << command_decode_p95 << "}\n";
  return state_encode_p95 >= 0.0 && state_encode_p95 < 50.0 && state_decode_p95 >= 0.0 &&
                 state_decode_p95 < 30.0 && command_encode_p95 >= 0.0 &&
                 command_encode_p95 < 30.0 && command_decode_p95 >= 0.0 && command_decode_p95 < 30.0
             ? 0
             : 1;
}
