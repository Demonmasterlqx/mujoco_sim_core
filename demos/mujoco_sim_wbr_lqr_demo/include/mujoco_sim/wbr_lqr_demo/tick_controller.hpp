#pragma once

#include <optional>
#include <vector>

#include <control/msgs.hpp>
#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/v1/sync.pb.h>
#include <mujoco_sim/wbr_lqr_demo/config.hpp>
#include <mujoco_sim/wbr_lqr_demo/controller_pipeline.hpp>
#include <mujoco_sim/wbr_lqr_demo/input_script.hpp>
#include <mujoco_sim/wbr_lqr_demo/interface_mapping.hpp>

namespace mujoco_sim::wbr_lqr_demo {

mujoco_sim::adapter_sdk::AdapterOptions MakeAdapterOptions(
    const AdapterConfig& config);

class TickController {
 public:
  TickController(const mujoco_sim::v1::ResourceLayout& registered_layout,
                 PipelineConfig pipeline_config = {},
                 InputScript input_script = {});
  TickController(const mujoco_sim::v1::ResourceLayout& registered_layout,
                 InterfaceConfig interface_config,
                 PipelineConfig pipeline_config,
                 InputScript input_script);

  std::vector<mujoco_sim::adapter_sdk::CommandValue> HandleTick(
      const mujoco_sim::v1::Tick& tick);
  std::vector<mujoco_sim::adapter_sdk::CommandValue> HandleTick(
      const mujoco_sim::v1::Tick& tick,
      const control::input_snapshot_t& input);

  void Reset();
  const std::optional<PipelineOutput>& last_output() const {
    return last_output_;
  }
  const control::input_snapshot_t& last_input() const { return last_input_; }
  std::size_t last_clamp_count() const {
    return command_mapper_.last_clamp_count();
  }

 private:
  TickDecoder decoder_;
  TorqueCommandMapper command_mapper_;
  ControllerPipeline pipeline_;
  InputScript input_script_;
  control::input_snapshot_t last_input_{};
  std::optional<PipelineOutput> last_output_;
};

}  // namespace mujoco_sim::wbr_lqr_demo
