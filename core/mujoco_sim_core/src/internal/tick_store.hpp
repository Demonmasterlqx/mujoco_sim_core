#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include <mujoco_sim/core/simulation_core.hpp>

namespace mujoco_sim::core::internal {

class TickStore {
 public:
  IssuedTickHandle Insert(const std::string& adapter_id,
                          mujoco_sim::v2::Tick message,
                          std::size_t layout_bytes_avoided,
                          std::uint64_t session_id, std::uint64_t epoch,
                          std::uint64_t tick_id);
  IssuedTickHandle Find(const std::string& adapter_id) const;
  bool IsCurrent(const IssuedTickIdentity& identity) const;
  void Invalidate(const std::string& adapter_id);
  void InvalidateAll();

 private:
  std::unordered_map<std::string, IssuedTickHandle> issued_;
  std::uint64_t next_issue_generation_ = 1;
};

}  // namespace mujoco_sim::core::internal
