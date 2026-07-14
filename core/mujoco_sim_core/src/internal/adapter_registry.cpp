#include "adapter_registry.hpp"

#include <utility>

namespace mujoco_sim::core::internal {

std::uint64_t AdapterRegistry::AllocateSession() noexcept {
  return next_session_id_++;
}

void AdapterRegistry::Clear() {
  controllers_.clear();
  observers_.clear();
  actuator_owners_.clear();
}

ControllerRecord* AdapterRegistry::FindController(const std::string& id) {
  const auto found = controllers_.find(id);
  return found == controllers_.end() ? nullptr : &found->second;
}

const ControllerRecord* AdapterRegistry::FindController(
    const std::string& id) const {
  const auto found = controllers_.find(id);
  return found == controllers_.end() ? nullptr : &found->second;
}

ObserverRecord* AdapterRegistry::FindObserver(const std::string& id) {
  const auto found = observers_.find(id);
  return found == observers_.end() ? nullptr : &found->second;
}

const ObserverRecord* AdapterRegistry::FindObserver(
    const std::string& id) const {
  const auto found = observers_.find(id);
  return found == observers_.end() ? nullptr : &found->second;
}

bool AdapterRegistry::HasController(const std::string& id) const {
  return controllers_.contains(id);
}

bool AdapterRegistry::HasObserver(const std::string& id) const {
  return observers_.contains(id);
}

void AdapterRegistry::InsertController(ControllerRecord controller) {
  controllers_.insert_or_assign(controller.id, std::move(controller));
}

void AdapterRegistry::InsertObserver(ObserverRecord observer) {
  observers_.insert_or_assign(observer.id, std::move(observer));
}

bool AdapterRegistry::EraseController(const std::string& id) {
  return controllers_.erase(id) != 0;
}

bool AdapterRegistry::EraseObserver(const std::string& id) {
  return observers_.erase(id) != 0;
}

std::vector<ControllerRecord*> AdapterRegistry::Controllers() {
  std::vector<ControllerRecord*> result;
  result.reserve(controllers_.size());
  for (auto& [_, controller] : controllers_) result.push_back(&controller);
  return result;
}

std::vector<const ControllerRecord*> AdapterRegistry::Controllers() const {
  std::vector<const ControllerRecord*> result;
  result.reserve(controllers_.size());
  for (const auto& [_, controller] : controllers_) result.push_back(&controller);
  return result;
}

std::vector<ObserverRecord*> AdapterRegistry::Observers() {
  std::vector<ObserverRecord*> result;
  result.reserve(observers_.size());
  for (auto& [_, observer] : observers_) result.push_back(&observer);
  return result;
}

std::vector<const ObserverRecord*> AdapterRegistry::Observers() const {
  std::vector<const ObserverRecord*> result;
  result.reserve(observers_.size());
  for (const auto& [_, observer] : observers_) result.push_back(&observer);
  return result;
}

std::size_t AdapterRegistry::ControllerCount() const noexcept {
  return controllers_.size();
}

std::size_t AdapterRegistry::ObserverCount() const noexcept {
  return observers_.size();
}

const std::string* AdapterRegistry::ActuatorOwner(
    const std::string& actuator) const {
  const auto found = actuator_owners_.find(actuator);
  return found == actuator_owners_.end() ? nullptr : &found->second;
}

void AdapterRegistry::ClaimActuator(const std::string& actuator,
                                    const std::string& controller_id) {
  actuator_owners_.insert_or_assign(actuator, controller_id);
}

void AdapterRegistry::ReleaseActuator(const std::string& actuator,
                                      const std::string& controller_id) {
  const auto found = actuator_owners_.find(actuator);
  if (found != actuator_owners_.end() && found->second == controller_id) {
    actuator_owners_.erase(found);
  }
}

}  // namespace mujoco_sim::core::internal
