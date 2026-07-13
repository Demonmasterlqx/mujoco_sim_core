#pragma once

#include <cstddef>
#include <future>
#include <variant>

#include <mujoco_sim/v2/sync.pb.h>

namespace mujoco_sim::server {

template <typename Request, typename Response>
struct RpcEvent {
  using response_type = Response;

  Request request;
  std::promise<Response> completion;
};

using GetAvailableResourcesEvent =
    RpcEvent<mujoco_sim::v2::GetAvailableResourcesRequest,
             mujoco_sim::v2::GetAvailableResourcesResponse>;
using RegisterAdapterEvent =
    RpcEvent<mujoco_sim::v2::RegisterAdapterRequest,
             mujoco_sim::v2::RegisterAdapterResponse>;
using ActivateAdapterEvent =
    RpcEvent<mujoco_sim::v2::ActivateAdapterRequest,
             mujoco_sim::v2::ActivateAdapterResponse>;
using UnregisterAdapterEvent =
    RpcEvent<mujoco_sim::v2::UnregisterAdapterRequest,
             mujoco_sim::v2::UnregisterAdapterResponse>;
using GetSensorStreamsEvent =
    RpcEvent<mujoco_sim::v2::GetSensorStreamsRequest,
             mujoco_sim::v2::GetSensorStreamsResponse>;

struct CommitEvent {
  mujoco_sim::v2::AdapterCommit message;
};

struct HeartbeatEvent {
  mujoco_sim::v2::AdapterHeartbeat message;
};

using ServerEvent =
    std::variant<GetAvailableResourcesEvent, RegisterAdapterEvent,
                 ActivateAdapterEvent, UnregisterAdapterEvent,
                 GetSensorStreamsEvent, CommitEvent, HeartbeatEvent>;

std::size_t EventPayloadBytes(const ServerEvent& event);
void CancelRpcEvent(ServerEvent* event, const char* reason);

}  // namespace mujoco_sim::server
