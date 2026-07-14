#pragma once

#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::adapter_sdk::internal {

inline bool SameTickStream(const mujoco_sim::v2::Tick& lhs,
                           const mujoco_sim::v2::Tick& rhs) {
  return lhs.adapter_id() == rhs.adapter_id() &&
         lhs.header().server_name() == rhs.header().server_name() &&
         lhs.header().session_id() == rhs.header().session_id() &&
         lhs.header().epoch() == rhs.header().epoch() &&
         lhs.header().protocol_major() == rhs.header().protocol_major() &&
         lhs.header().protocol_minor() == rhs.header().protocol_minor();
}

inline bool SameTickIdentity(const mujoco_sim::v2::Tick& lhs,
                             const mujoco_sim::v2::Tick& rhs) {
  return SameTickStream(lhs, rhs) &&
         lhs.header().tick_id() == rhs.header().tick_id();
}

inline bool SameTickPayload(const mujoco_sim::v2::Tick& lhs,
                            const mujoco_sim::v2::Tick& rhs) {
  auto normalized_lhs = lhs;
  auto normalized_rhs = rhs;
  normalized_lhs.mutable_header()->set_timestamp_unix_nanos(0);
  normalized_rhs.mutable_header()->set_timestamp_unix_nanos(0);
  return normalized_lhs.SerializeAsString() == normalized_rhs.SerializeAsString();
}

}  // namespace mujoco_sim::adapter_sdk::internal
