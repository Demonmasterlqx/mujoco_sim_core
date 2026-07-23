#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include <control/msgs.hpp>
#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <mujoco_sim/wbr_lqr_demo/config.hpp>
#include <mujoco_sim/wbr_lqr_demo/controller_pipeline.hpp>
#include <mujoco_sim/wbr_lqr_demo/input_script.hpp>
#include <mujoco_sim/wbr_lqr_demo/interface_mapping.hpp>

namespace mujoco_sim::wbr_lqr_demo {

mujoco_sim::adapter_sdk::ControllerOptions MakeControllerOptions(const AdapterConfig& config);

class WbrController {
 public:
  WbrController(const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout,
                PipelineConfig pipeline_config = {}, InputScript input_script = {});
  WbrController(const mujoco_sim::adapter_sdk::ControllerLayout& registered_layout,
                InterfaceConfig interface_config, PipelineConfig pipeline_config,
                InputScript input_script);

  std::vector<double> ComputeCommand(const mujoco_sim::adapter_sdk::StateLease& state);
  std::vector<double> ComputeCommand(const mujoco_sim::adapter_sdk::StateLease& state,
                                     const control::input_snapshot_t& input);
  std::vector<double> ComputeCommand(std::span<const double> observations,
                                     double simulation_time_s);
  std::vector<double> ComputeCommand(std::span<const double> observations, double simulation_time_s,
                                     const control::input_snapshot_t& input);

  void Reset();
  const std::optional<PipelineOutput>& last_output() const { return last_output_; }
  const control::input_snapshot_t& last_input() const { return last_input_; }
  std::size_t last_clamp_count() const { return command_mapper_.last_clamp_count(); }

 private:
  StateDecoder decoder_;
  TorqueCommandMapper command_mapper_;
  ControllerPipeline pipeline_;
  InputScript input_script_;
  control::input_snapshot_t last_input_{};
  std::optional<PipelineOutput> last_output_;
};

}  // namespace mujoco_sim::wbr_lqr_demo
