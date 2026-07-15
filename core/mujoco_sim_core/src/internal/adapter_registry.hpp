#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco_sim/core/domain.hpp>


#include "controller_state_machine.hpp"

namespace mujoco_sim::core::internal {

struct ControllerRecord {
  std::string id;
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
  std::uint32_t commit_timeout_ms = 1000;
  std::uint32_t session_timeout_ms = 1000;
  ControllerPhase phase = ControllerPhase::kProvisional;
  mujoco_sim::core::ResourceLayout layout;
  std::chrono::steady_clock::time_point registered_at =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_presence_at = registered_at;
  std::optional<std::uint64_t> last_heartbeat_tick;
  std::optional<std::uint64_t> last_tick_sent;
  std::uint64_t state_since_tick = 0;
  std::uint64_t timed_out_since_tick = 0;
  std::uint64_t timeout_count = 0;
  std::optional<mujoco_sim::core::AdapterCommit> recovery_proof;
  std::optional<mujoco_sim::core::AdapterCommit> last_commit;
  std::string message;
};

struct ObserverRecord {
  std::string id;
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
  std::uint32_t session_timeout_ms = 1000;
  ObserverPhase phase = ObserverPhase::kProvisional;
  mujoco_sim::core::ResourceLayout layout;
  mujoco_sim::core::ObservationDelivery delivery;
  std::chrono::steady_clock::time_point registered_at =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_heartbeat_at = registered_at;
  std::uint64_t state_since_tick = 0;
  std::uint64_t activation_tick = 0;
  std::uint64_t timeout_count = 0;
  std::uint64_t sequence = 0;
  std::optional<std::uint64_t> last_delivery_tick;
  std::uint64_t delivery_phase_tick = 0;
  std::uint64_t next_delivery_period = 1;
  std::string message;
};

class AdapterRegistry {
 public:
  std::uint64_t AllocateSession() noexcept;
  void Clear();

  ControllerRecord* FindController(const std::string& id);
  const ControllerRecord* FindController(const std::string& id) const;
  ControllerRecord* FindControllerBySession(std::uint64_t session_id);
  ObserverRecord* FindObserver(const std::string& id);
  const ObserverRecord* FindObserver(const std::string& id) const;
  ObserverRecord* FindObserverBySession(std::uint64_t session_id);
  bool HasController(const std::string& id) const;
  bool HasObserver(const std::string& id) const;

  void InsertController(ControllerRecord controller);
  void InsertObserver(ObserverRecord observer);
  bool EraseController(const std::string& id);
  bool EraseObserver(const std::string& id);

  std::vector<ControllerRecord*> Controllers();
  std::vector<const ControllerRecord*> Controllers() const;
  std::vector<ObserverRecord*> Observers();
  std::vector<const ObserverRecord*> Observers() const;
  std::size_t ControllerCount() const noexcept;
  std::size_t ObserverCount() const noexcept;

  const std::string* ActuatorOwner(const std::string& actuator) const;
  void ClaimActuator(const std::string& actuator,
                     const std::string& controller_id);
  void ReleaseActuator(const std::string& actuator,
                       const std::string& controller_id);

 private:
  std::unordered_map<std::string, ControllerRecord> controllers_;
  std::unordered_map<std::string, ObserverRecord> observers_;
  std::unordered_map<std::uint64_t, std::string> controller_sessions_;
  std::unordered_map<std::uint64_t, std::string> observer_sessions_;
  std::unordered_map<std::string, std::string> actuator_owners_;
  std::uint64_t next_session_id_ = 1;
};

}  // namespace mujoco_sim::core::internal
