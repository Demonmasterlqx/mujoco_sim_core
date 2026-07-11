#pragma once

#include <string>

namespace mujoco_sim::protocol {

inline constexpr unsigned int kProtocolMajor = 1;
inline constexpr unsigned int kProtocolMinor = 1;

inline std::string RegistryServiceName(const std::string& server_name) {
  return "mujoco_sim." + server_name + ".registry";
}

inline std::string TickTopicName(const std::string& server_name,
                                 const std::string& controller_id) {
  return "mujoco_sim/" + server_name + "/tick/" + controller_id;
}

inline std::string CommitTopicName(const std::string& server_name) {
  return "mujoco_sim/" + server_name + "/commit";
}

inline std::string HeartbeatTopicName(const std::string& server_name) {
  return "mujoco_sim/" + server_name + "/heartbeat";
}

inline std::string StatusTopicName(const std::string& server_name) {
  return "mujoco_sim/" + server_name + "/status";
}

inline constexpr const char* kGetAvailableResourcesMethod =
    "GetAvailableResources";
inline constexpr const char* kRegisterControllerMethod = "RegisterController";
inline constexpr const char* kActivateControllerMethod = "ActivateController";
inline constexpr const char* kUnregisterControllerMethod =
    "UnregisterController";

}  // namespace mujoco_sim::protocol
