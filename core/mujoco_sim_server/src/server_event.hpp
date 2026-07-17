#pragma once

#include <cstddef>
#include <future>
#include <variant>

#include <mujoco_sim/core/domain.hpp>
#include <mujoco_sim/v4/sync.pb.h>

namespace mujoco_sim::server {

template <typename Request, typename Response>
struct RpcEvent {
  using response_type = Response;

  Request request;
  std::promise<Response> completion;
};

using GetAvailableResourcesEvent = RpcEvent<mujoco_sim::v4::GetAvailableResourcesRequest,
                                            mujoco_sim::v4::GetAvailableResourcesResponse>;
using RegisterAdapterEvent =
    RpcEvent<mujoco_sim::v4::RegisterAdapterRequest, mujoco_sim::v4::RegisterAdapterResponse>;
using ActivateAdapterEvent =
    RpcEvent<mujoco_sim::v4::ActivateAdapterRequest, mujoco_sim::v4::ActivateAdapterResponse>;
using UnregisterAdapterEvent =
    RpcEvent<mujoco_sim::v4::UnregisterAdapterRequest, mujoco_sim::v4::UnregisterAdapterResponse>;
using GetSensorStreamsEvent =
    RpcEvent<mujoco_sim::v4::GetSensorStreamsRequest, mujoco_sim::v4::GetSensorStreamsResponse>;

struct HeartbeatEvent {
  mujoco_sim::core::AdapterHeartbeat message;
};

using ServerEvent =
    std::variant<GetAvailableResourcesEvent, RegisterAdapterEvent, ActivateAdapterEvent,
                 UnregisterAdapterEvent, GetSensorStreamsEvent, HeartbeatEvent>;

std::size_t EventPayloadBytes(const ServerEvent& event);
void CancelRpcEvent(ServerEvent* event, const char* reason);

}  // namespace mujoco_sim::server
