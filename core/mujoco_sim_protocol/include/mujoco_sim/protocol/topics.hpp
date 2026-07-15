#pragma once

#include <string>

namespace mujoco_sim::protocol {

inline constexpr unsigned int kProtocolMajor = 3;
inline constexpr unsigned int kProtocolMinor = 0;

inline std::string RegistryServiceName(const std::string& server_name) {
  return "mujoco_sim.v3." + server_name + ".registry";
}

inline std::string TickTopicName(const std::string& server_name,
                                 const std::string& adapter_id) {
  return "mujoco_sim/v3/" + server_name + "/tick/" + adapter_id;
}

inline std::string CommitTopicName(const std::string& server_name) {
  return "mujoco_sim/v3/" + server_name + "/commit";
}

inline std::string StatusTopicName(const std::string& server_name) {
  return "mujoco_sim/v3/" + server_name + "/status";
}

inline std::string ObservationTopicName(const std::string& server_name,
                                        const std::string& adapter_id) {
  return "mujoco_sim/v3/" + server_name + "/observation/" + adapter_id;
}

inline std::string SessionHeartbeatTopicName(const std::string& server_name) {
  return "mujoco_sim/v3/" + server_name + "/session_heartbeat";
}

inline std::string ControllerFaultTopicName(const std::string& server_name) {
  return "mujoco_sim/v3/" + server_name + "/controller_fault";
}

inline std::string SensorStreamTopicName(const std::string& server_name,
                                         const std::string& stream_name) {
  return "mujoco_sim/v3/" + server_name + "/sensor/" + stream_name;
}

inline constexpr const char* kGetAvailableResourcesMethod =
    "GetAvailableResources";
inline constexpr const char* kRegisterAdapterMethod = "RegisterAdapter";
inline constexpr const char* kActivateAdapterMethod = "ActivateAdapter";
inline constexpr const char* kUnregisterAdapterMethod = "UnregisterAdapter";
inline constexpr const char* kGetSensorStreamsMethod = "GetSensorStreams";

}  // namespace mujoco_sim::protocol
