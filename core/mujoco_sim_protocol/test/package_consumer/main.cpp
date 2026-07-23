#include <mujoco_sim/protocol/data_frame.hpp>
#include <mujoco_sim/v4/sync.pb.h>

int main() {
  mujoco_sim::protocol::DataFrameHeader header;
  mujoco_sim::v4::ServerStatus status;
  return header.kind == mujoco_sim::protocol::DataFrameKind::kControllerState &&
                 status.tick_id() == 0
             ? 0
             : 1;
}
