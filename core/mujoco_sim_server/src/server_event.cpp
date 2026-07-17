#include "server_event.hpp"

#include <string>
#include <type_traits>

namespace mujoco_sim::server {

namespace {

template <typename Response>
Response StoppedResponse(const char* reason) {
  Response response;
  response.set_code(mujoco_sim::v4::INTERNAL_ERROR);
  response.set_message(reason == nullptr ? "server is stopping" : reason);
  return response;
}

}  // namespace

std::size_t EventPayloadBytes(const ServerEvent& event) {
  return std::visit(
      [](const auto& value) -> std::size_t {
        using Event = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<Event, HeartbeatEvent>) {
          return sizeof(value.message);
        } else {
          return value.request.ByteSizeLong();
        }
      },
      event);
}

void CancelRpcEvent(ServerEvent* event, const char* reason) {
  if (event == nullptr) return;
  std::visit(
      [reason](auto& value) {
        using Event = std::decay_t<decltype(value)>;
        if constexpr (!std::is_same_v<Event, HeartbeatEvent>) {
          using Response = typename Event::response_type;
          value.completion.set_value(StoppedResponse<Response>(reason));
        }
      },
      *event);
}

}  // namespace mujoco_sim::server
