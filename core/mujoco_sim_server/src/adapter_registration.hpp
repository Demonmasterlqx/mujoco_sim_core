#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <string_view>

#include <mujoco_sim/core/simulation_core.hpp>

#include "controller_command_inbox.hpp"

namespace mujoco_sim::server {

struct PublisherPreparation {
  bool ready = false;
  bool created = false;
  std::string error;
};

using PreparePublisher =
    std::function<PublisherPreparation(std::string_view, mujoco_sim::core::AdapterRole)>;
using RollbackPublisher = std::function<void(std::string_view, mujoco_sim::core::AdapterRole)>;

mujoco_sim::core::EngineResult<mujoco_sim::core::RegisterAdapterResponse> RegisterAdapterAtomically(
    mujoco_sim::core::SimulationCore* core, ControllerCommandInbox* command_inbox,
    const mujoco_sim::core::RegisterAdapterRequest& request,
    std::chrono::steady_clock::time_point now, const PreparePublisher& prepare_publisher,
    const RollbackPublisher& rollback_publisher);

}  // namespace mujoco_sim::server
