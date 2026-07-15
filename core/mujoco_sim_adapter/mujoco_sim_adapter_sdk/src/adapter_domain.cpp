#include <mujoco_sim/adapter_sdk/adapter_domain.hpp>

#include <utility>

namespace mujoco_sim::adapter_sdk {

ControllerLayout::ControllerLayout(
    std::vector<ResourceInfo> observations, std::vector<ResourceInfo> commands,
    std::size_t observation_scalar_count, std::size_t command_scalar_count)
    : observations_(std::move(observations)),
      commands_(std::move(commands)),
      observation_scalar_count_(observation_scalar_count),
      command_scalar_count_(command_scalar_count) {
  for (const auto& resource : observations_) {
    observation_index_.emplace(resource.name, resource.scalars);
  }
  for (const auto& resource : commands_) {
    command_index_.emplace(resource.name, resource.scalars);
  }
}

std::optional<ScalarSpan> ControllerLayout::Observation(
    std::string_view name) const {
  const auto found = observation_index_.find(std::string(name));
  return found == observation_index_.end() ? std::nullopt
                                            : std::optional(found->second);
}

std::optional<ScalarSpan> ControllerLayout::Command(
    std::string_view name) const {
  const auto found = command_index_.find(std::string(name));
  return found == command_index_.end() ? std::nullopt
                                        : std::optional(found->second);
}

std::size_t ControllerLayout::observation_scalar_count() const noexcept {
  return observation_scalar_count_;
}

std::size_t ControllerLayout::command_scalar_count() const noexcept {
  return command_scalar_count_;
}

const std::vector<ResourceInfo>& ControllerLayout::observations() const noexcept {
  return observations_;
}

const std::vector<ResourceInfo>& ControllerLayout::commands() const noexcept {
  return commands_;
}

}  // namespace mujoco_sim::adapter_sdk
