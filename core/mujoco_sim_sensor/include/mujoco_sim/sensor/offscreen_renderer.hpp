#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco_sim/sensor/source_profile.hpp>

namespace mujoco_sim::sensor {

enum class OffscreenBackend {
  kEglSurfaceless,
  kEglPbuffer,
  kOsMesa,
};

enum class BackendPreference {
  kAutomatic,
  kEglOnly,
  kOsMesaOnly,
};

struct OffscreenRendererOptions {
  BackendPreference backend = BackendPreference::kAutomatic;
};

std::string_view ToString(OffscreenBackend backend) noexcept;

class OffscreenRenderer {
 public:
  static std::unique_ptr<OffscreenRenderer> Create(
      const mujoco_sim::core::SimulationSnapshot& snapshot,
      CameraProfile profile,
      OffscreenRendererOptions options,
      std::string* error_message);

  static std::unique_ptr<OffscreenRenderer> Create(
      const mujoco_sim::core::SimulationSnapshot& snapshot,
      CameraProfile profile,
      std::string* error_message) {
    return Create(snapshot, std::move(profile), {}, error_message);
  }

  ~OffscreenRenderer();
  OffscreenRenderer(OffscreenRenderer&&) noexcept;
  OffscreenRenderer& operator=(OffscreenRenderer&&) noexcept;

  OffscreenRenderer(const OffscreenRenderer&) = delete;
  OffscreenRenderer& operator=(const OffscreenRenderer&) = delete;

  const CameraProfile& profile() const noexcept;
  const std::vector<mujoco_sim::v2::StreamDescriptor>& descriptors()
      const noexcept;
  OffscreenBackend backend() const noexcept;

  bool Capture(const mujoco_sim::core::SimulationSnapshot& snapshot,
               const CaptureMetadata& metadata,
               std::vector<RawFrame>* frames,
               std::string* error_message);

 private:
  struct Impl;
  explicit OffscreenRenderer(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim::sensor
