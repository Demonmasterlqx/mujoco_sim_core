#pragma once

#include <memory>
#include <string>

#include <mujoco_sim/sensor/source_profile.hpp>

namespace mujoco_sim::sensor {

class RayLidar {
 public:
  static std::unique_ptr<RayLidar> Create(
      const mujoco_sim::core::SimulationSnapshot& snapshot,
      RayLidarProfile profile,
      std::string* error_message);

  ~RayLidar();
  RayLidar(RayLidar&&) noexcept;
  RayLidar& operator=(RayLidar&&) noexcept;

  RayLidar(const RayLidar&) = delete;
  RayLidar& operator=(const RayLidar&) = delete;

  const RayLidarProfile& profile() const noexcept;
  const mujoco_sim::v2::StreamDescriptor& descriptor() const noexcept;

  bool Capture(const mujoco_sim::core::SimulationSnapshot& snapshot,
               const CaptureMetadata& metadata,
               RawFrame* frame,
               std::string* error_message) const;

 private:
  struct Impl;
  explicit RayLidar(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::sensor
