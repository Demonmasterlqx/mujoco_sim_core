#include "tick_store.hpp"

#include <atomic>
#include <memory>
#include <utility>

namespace mujoco_sim::core::internal {

IssuedTickHandle TickStore::Insert(
    const std::string& adapter_id, mujoco_sim::core::Tick message,
    std::uint64_t session_id, std::uint64_t epoch, std::uint64_t tick_id) {
  auto issued_tick = std::make_shared<IssuedTick>();
  issued_tick->frame = std::move(message);
  issued_tick->identity = {adapter_id, session_id, epoch, tick_id,
                           next_issue_generation_++};
  issued_.insert_or_assign(adapter_id, issued_tick);
  return issued_tick;
}

IssuedTickHandle TickStore::Find(const std::string& adapter_id) const {
  const auto found = issued_.find(adapter_id);
  return found == issued_.end() ? IssuedTickHandle{} : found->second;
}

bool TickStore::IsCurrent(const IssuedTickIdentity& identity) const {
  const auto found = issued_.find(identity.adapter_id);
  return found != issued_.end() &&
         found->second->current.load(std::memory_order_acquire) &&
         found->second->identity.session_id == identity.session_id &&
         found->second->identity.epoch == identity.epoch &&
         found->second->identity.tick_id == identity.tick_id &&
         found->second->identity.issue_generation == identity.issue_generation;
}

void TickStore::Invalidate(const std::string& adapter_id) {
  const auto found = issued_.find(adapter_id);
  if (found == issued_.end()) return;
  found->second->current.store(false, std::memory_order_release);
  issued_.erase(found);
}

void TickStore::InvalidateAll() {
  for (const auto& [_, tick] : issued_) {
    tick->current.store(false, std::memory_order_release);
  }
  issued_.clear();
}

}  // namespace mujoco_sim::core::internal
