#include <mujoco_sim/sensor/sensor.hpp>

#include <string>

int main() {
  mujoco_sim::sensor::StreamShape shape;
  std::string error;
  if (!mujoco_sim::sensor::StreamShapeFor(mujoco_sim::v4::RGB8, 64, 48, &shape, &error)) {
    return 1;
  }
  if (shape.row_stride_bytes != 192U || shape.payload_bytes != 9216U) {
    return 1;
  }
  return 0;
}
