#pragma once

#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <mujoco_sim/sensor_demo/icp_mapper.hpp>

namespace mujoco_sim::sensor_demo {

struct MapOutputConfig {
  std::string pcd_filename = "map.pcd";
  std::string trajectory_filename = "trajectory.csv";
  std::string top_down_png_filename = "map_top_down.png";
  std::size_t image_width = 1024;
  std::size_t image_height = 768;
  double padding_meters = 0.5;
};

struct MapOutputPaths {
  std::filesystem::path pcd;
  std::filesystem::path trajectory_csv;
  std::filesystem::path top_down_png;
};

class MapOutputError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// All data is validated before the output directory or any temporary file is
// created. Final files are replaced only after all three staged writes succeed.
MapOutputPaths SaveMapOutputs(const std::filesystem::path& output_directory, const PointCloud& map,
                              const MapperTrajectory& trajectory,
                              const MapOutputConfig& config = {});

MapOutputPaths SaveMapOutputs(const std::filesystem::path& output_directory,
                              const SimpleIcpMapper& mapper, const MapOutputConfig& config = {});

}  // namespace mujoco_sim::sensor_demo
