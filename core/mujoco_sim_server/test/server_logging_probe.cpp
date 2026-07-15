#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <ecal/ecal.h>
#include <ecal/pubsub/publisher.h>
#include <ecal/service/client.h>
#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v3/sync.pb.h>

namespace {

eCAL::SServiceMethodInformation MethodInfo(const std::string& method) {
  eCAL::SServiceMethodInformation info;
  info.method_name = method;
  return info;
}

bool Call(eCAL::CServiceClient* client, const std::string& method,
          const std::string& request) {
  eCAL::ServiceResponseVecT responses;
  if (!client->CallWithResponse(method, request, responses, 1000)) {
    return false;
  }
  for (const auto& response : responses) {
    if (response.call_state == eCAL::eCallState::executed) {
      return true;
    }
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: server_logging_probe SERVER_NAME\n";
    return 1;
  }
  const std::string server_name = argv[1];
  if (!eCAL::Initialize("mujoco-sim-server-logging-probe")) {
    std::cerr << "failed to initialize eCAL\n";
    return 1;
  }

  int result = 0;
  {
    eCAL::ServiceMethodInformationSetT methods;
    methods.insert(
        MethodInfo(mujoco_sim::protocol::kRegisterAdapterMethod));
    eCAL::CServiceClient client(
        mujoco_sim::protocol::RegistryServiceName(server_name), methods);
    eCAL::CPublisher commit(
        mujoco_sim::protocol::CommitTopicName(server_name));
    eCAL::CPublisher heartbeat(
        mujoco_sim::protocol::SessionHeartbeatTopicName(server_name));

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(5);
    while (!client.IsConnected() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!client.IsConnected()) {
      std::cerr << "registry service was not discovered\n";
      result = 1;
    } else {
      const auto topic_deadline = std::chrono::steady_clock::now() +
                                  std::chrono::seconds(2);
      while ((commit.GetSubscriberCount() == 0 ||
              heartbeat.GetSubscriberCount() == 0) &&
             std::chrono::steady_clock::now() < topic_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      const std::string malformed(1, static_cast<char>(0xff));
      mujoco_sim::v3::RegisterAdapterRequest rejected;
      rejected.set_role(mujoco_sim::v3::CONTROLLER);
      rejected.add_command_names("hinge_motor");
      const std::string rejected_bytes = rejected.SerializeAsString();

      for (int index = 0; index < 3; ++index) {
        if (!Call(&client,
                  mujoco_sim::protocol::kRegisterAdapterMethod,
                  malformed) ||
            !Call(&client,
                  mujoco_sim::protocol::kRegisterAdapterMethod,
                  rejected_bytes)) {
          result = 1;
        }
        commit.Send(malformed);
        heartbeat.Send(malformed);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1100));
      Call(&client, mujoco_sim::protocol::kRegisterAdapterMethod,
           malformed);
      Call(&client, mujoco_sim::protocol::kRegisterAdapterMethod,
           rejected_bytes);
      commit.Send(malformed);
      heartbeat.Send(malformed);

      const auto traffic_deadline = std::chrono::steady_clock::now() +
                                    std::chrono::seconds(2);
      while (std::chrono::steady_clock::now() < traffic_deadline) {
        Call(&client, mujoco_sim::protocol::kRegisterAdapterMethod,
             malformed);
        commit.Send(malformed);
        heartbeat.Send(malformed);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }
  eCAL::Finalize();
  return result;
}
