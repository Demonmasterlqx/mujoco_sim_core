#include <mujoco_sim/adapter_sdk/adapter_domain.hpp>

#include <string>
#include <vector>

int main() {
  std::vector<mujoco_sim::adapter_sdk::ResourceInfo> observations{
      {.name = "joint_position", .scalars = {.offset = 0, .count = 1}}};
  mujoco_sim::adapter_sdk::ControllerLayout layout(std::move(observations), {}, 1, 0);
  const auto span = layout.Observation("joint_position");
  return span && span->offset == 0 && span->count == 1 ? 0 : 1;
}
