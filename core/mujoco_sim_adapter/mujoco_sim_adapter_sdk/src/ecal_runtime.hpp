#pragma once

#include <memory>
#include <string>

namespace mujoco_sim::adapter_sdk::internal {

std::shared_ptr<void> AcquireEcalRuntime(const std::string& unit_name, std::string* error_message);

}  // namespace mujoco_sim::adapter_sdk::internal
