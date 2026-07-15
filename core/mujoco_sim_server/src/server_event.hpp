#pragma once

#include <cstddef>
#include <future>
#include <variant>

#include <mujoco_sim/core/domain.hpp>
#include <mujoco_sim/v3/sync.pb.h>

namespace mujoco_sim::server {

template <typename Request, typename Response>
struct RpcEvent {
  using response_type = Response;

  Request request;
  std::promise<Response> completion;
};

using GetAvailableResourcesEvent =
    RpcEvent<mujoco_sim::v3::GetAvailableResourcesRequest,
             mujoco_sim::v3::GetAvailableResourcesResponse>;
using RegisterAdapterEvent =
    RpcEvent<mujoco_sim::v3::RegisterAdapterRequest,
             mujoco_sim::v3::RegisterAdapterResponse>;
using ActivateAdapterEvent =
    RpcEvent<mujoco_sim::v3::ActivateAdapterRequest,
             mujoco_sim::v3::ActivateAdapterResponse>;
using UnregisterAdapterEvent =
    RpcEvent<mujoco_sim::v3::UnregisterAdapterRequest,
             mujoco_sim::v3::UnregisterAdapterResponse>;
using GetSensorStreamsEvent =
    RpcEvent<mujoco_sim::v3::GetSensorStreamsRequest,
             mujoco_sim::v3::GetSensorStreamsResponse>;

struct CommitEvent {
  mujoco_sim::core::AdapterCommit message;
};

struct HeartbeatEvent {
  mujoco_sim::core::AdapterHeartbeat message;
};

struct ControllerFaultEvent {
  mujoco_sim::core::ControllerFault message;
};

using ServerEvent =
    std::variant<GetAvailableResourcesEvent, RegisterAdapterEvent,
                 ActivateAdapterEvent, UnregisterAdapterEvent,
                 GetSensorStreamsEvent, CommitEvent, HeartbeatEvent,
                 ControllerFaultEvent>;

std::size_t EventPayloadBytes(const ServerEvent& event);
void CancelRpcEvent(ServerEvent* event, const char* reason);

}  // namespace mujoco_sim::server
