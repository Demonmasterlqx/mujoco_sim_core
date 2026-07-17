#include <mujoco_sim/wbr_lqr_demo/controller.hpp>

#include <algorithm>
#include <cstdint>
#include <utility>

namespace mujoco_sim::wbr_lqr_demo {

mujoco_sim::adapter_sdk::ControllerOptions MakeControllerOptions(const AdapterConfig& config) {
  mujoco_sim::adapter_sdk::ControllerOptions options;
  options.server_name = config.server_name;
  options.adapter_id = config.adapter_id;
  options.observation_names = WbrObservationNames(config.interfaces);
  options.command_names = WbrCommandNames(config.interfaces);
  options.service_timeout_ms = config.service_timeout_ms;
  return options;
}

WbrController::WbrController(const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout,
                             PipelineConfig pipeline_config, InputScript input_script)
    : WbrController(registered_layout, MakeUpstreamWbrInterfaceConfig(), std::move(pipeline_config),
                    std::move(input_script)) {}

WbrController::WbrController(const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout,
                             InterfaceConfig interface_config, PipelineConfig pipeline_config,
                             InputScript input_script)
    : decoder_(interface_config, registered_layout),
      command_mapper_(std::move(interface_config), registered_layout),
      pipeline_(std::move(pipeline_config)),
      input_script_(std::move(input_script)) {}

std::vector<double> WbrController::ComputeCommand(
    const mujoco_sim::adapter_sdk::StateLease& state) {
  return ComputeCommand(state, input_script_.InputAt(pipeline_.tick_count()));
}

std::vector<double> WbrController::ComputeCommand(const mujoco_sim::adapter_sdk::StateLease& state,
                                                  const control::input_snapshot_t& input) {
  const control::msg_raw_state_t raw = decoder_.Decode(state);
  last_input_ = input;
  last_output_ = pipeline_.Step(raw, input);
  return command_mapper_.Map(last_output_->raw_torque);
}

std::vector<double> WbrController::ComputeCommand(std::span<const double> observations,
                                                  double simulation_time_s) {
  return ComputeCommand(observations, simulation_time_s,
                        input_script_.InputAt(pipeline_.tick_count()));
}

std::vector<double> WbrController::ComputeCommand(std::span<const double> observations,
                                                  double simulation_time_s,
                                                  const control::input_snapshot_t& input) {
  const control::msg_raw_state_t raw = decoder_.Decode(observations, simulation_time_s);
  last_input_ = input;
  last_output_ = pipeline_.Step(raw, input);
  return command_mapper_.Map(last_output_->raw_torque);
}

void WbrController::Reset() {
  pipeline_.Reset();
  last_input_ = {};
  last_output_.reset();
}

}  // namespace mujoco_sim::wbr_lqr_demo
