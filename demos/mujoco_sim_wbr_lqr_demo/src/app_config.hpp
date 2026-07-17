#pragma once

#include <cstdint>
#include <string>

#include "wbr_lqr_generated_config.hpp"

#include <mujoco_sim/wbr_lqr_demo/config.hpp>
#include <mujoco_sim/wbr_lqr_demo/input_script.hpp>

namespace mujoco_sim::wbr_lqr_demo {

struct AppConfig {
  AdapterConfig adapter;
  PipelineConfig pipeline;
  InputScript input_script;
  std::uint64_t log_every_ticks = 0;
};

bool BuildAppConfig(const generated::WbrConfig& generated_config, AppConfig* config,
                    std::string* error_message);

}  // namespace mujoco_sim::wbr_lqr_demo
