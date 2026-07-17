#pragma once

#include <string>

namespace mujoco_sim::protocol {

inline constexpr unsigned int kProtocolMajor = 4;
inline constexpr unsigned int kProtocolMinor = 0;

inline std::string RegistryServiceName(const std::string& server_name) {
  return "mujoco_sim.v4." + server_name + ".registry";
}

inline std::string ControllerStateTopicName(const std::string& server_name,
                                            const std::string& adapter_id) {
  return "mujoco_sim/v4/" + server_name + "/controller_state/" + adapter_id;
}

inline std::string ControllerCommandTopicName(const std::string& server_name) {
  return "mujoco_sim/v4/" + server_name + "/controller_command";
}

inline std::string StatusTopicName(const std::string& server_name) {
  return "mujoco_sim/v4/" + server_name + "/status";
}

inline std::string ObservationTopicName(const std::string& server_name,
                                        const std::string& adapter_id) {
  return "mujoco_sim/v4/" + server_name + "/observation/" + adapter_id;
}

inline std::string SessionHeartbeatTopicName(const std::string& server_name) {
  return "mujoco_sim/v4/" + server_name + "/session_heartbeat";
}

inline std::string ControllerFaultTopicName(const std::string& server_name) {
  return "mujoco_sim/v4/" + server_name + "/controller_fault";
}

inline std::string SensorStreamTopicName(const std::string& server_name,
                                         const std::string& stream_name) {
  return "mujoco_sim/v4/" + server_name + "/sensor/" + stream_name;
}

inline constexpr const char* kGetAvailableResourcesMethod = "GetAvailableResources";
inline constexpr const char* kRegisterAdapterMethod = "RegisterAdapter";
inline constexpr const char* kActivateAdapterMethod = "ActivateAdapter";
inline constexpr const char* kUnregisterAdapterMethod = "UnregisterAdapter";
inline constexpr const char* kGetSensorStreamsMethod = "GetSensorStreams";

}  // namespace mujoco_sim::protocol
