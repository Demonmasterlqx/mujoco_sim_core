#include <mujoco_sim/adapter_sdk/adapter_domain.hpp>
#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>

#include <string>
#include <type_traits>
#include <vector>

int main() {
  using namespace mujoco_sim::adapter_sdk;
  static_assert(!std::is_copy_constructible_v<ControllerClient>);
  static_assert(!std::is_copy_constructible_v<ObserverClient>);
  static_assert(!std::is_copy_constructible_v<SensorStreamClient>);

  std::vector<mujoco_sim::adapter_sdk::ResourceInfo> observations{
      {.name = "joint_position", .scalars = {.offset = 0, .count = 1}}};
  ControllerLayout layout(std::move(observations), {}, 1, 0);
  const auto span = layout.Observation("joint_position");
  if (!span || span->offset != 0 || span->count != 1) return 1;

  ControllerOptions controller_options;
  controller_options.adapter_id = "consumer-controller";
  controller_options.command_names = {"actuator"};
  ControllerClient controller(std::move(controller_options));

  ObserverOptions observer_options;
  observer_options.observer_id = "consumer-observer";
  observer_options.observation_names = {"joint_position"};
  ObserverClient observer(std::move(observer_options));

  SensorStreamOptions sensor_options;
  sensor_options.stream_names = {"camera"};
  SensorStreamClient sensor(std::move(sensor_options));
  return 0;
}
