#include <mujoco_sim/sensor/ray_lidar.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "internal.hpp"

namespace mujoco_sim::sensor {
namespace {

constexpr double kTwoPi = 6.28318530717958647692528676655900577;

}  // namespace

struct RayLidar::Impl {
  RayLidarProfile profile;
  mujoco_sim::v4::StreamDescriptor descriptor;
  std::vector<double> vertical_angles;
  const mjModel* model = nullptr;
  int site_id = -1;
};

RayLidar::RayLidar(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
RayLidar::~RayLidar() = default;
RayLidar::RayLidar(RayLidar&&) noexcept = default;
RayLidar& RayLidar::operator=(RayLidar&&) noexcept = default;

std::unique_ptr<RayLidar> RayLidar::Create(const core::SimulationSnapshot& snapshot,
                                           RayLidarProfile profile, std::string* error_message) {
  auto impl = std::make_unique<Impl>();
  impl->profile = std::move(profile);
  if (!detail::BuildRayLidarDescriptor(snapshot, impl->profile, &impl->descriptor,
                                       &impl->vertical_angles, &impl->site_id, error_message)) {
    return nullptr;
  }
  impl->model = snapshot.model();
  return std::unique_ptr<RayLidar>(new RayLidar(std::move(impl)));
}

const RayLidarProfile& RayLidar::profile() const noexcept {
  return impl_->profile;
}

const mujoco_sim::v4::StreamDescriptor& RayLidar::descriptor() const noexcept {
  return impl_->descriptor;
}

bool RayLidar::Capture(const core::SimulationSnapshot& snapshot, const CaptureMetadata& metadata,
                       RawFrame* frame, std::string* error_message) const {
  detail::ClearError(error_message);
  if (frame == nullptr) {
    return detail::Fail("ray lidar frame output is null", error_message);
  }
  frame->header.Clear();
  frame->payload.clear();
  if (!detail::ValidateCaptureMetadata(metadata, error_message)) {
    return false;
  }
  if (snapshot.model() != impl_->model || snapshot.data() == nullptr) {
    return detail::Fail("ray lidar snapshot is not the model used to create this source",
                        error_message);
  }

  const std::size_t width = impl_->profile.horizontal_samples;
  const std::size_t height = impl_->vertical_angles.size();
  const std::size_t ray_count = width * height;
  const mjData* data = snapshot.data();
  const mjtNum* site_position = data->site_xpos + 3 * impl_->site_id;
  const mjtNum* site_rotation = data->site_xmat + 9 * impl_->site_id;
  const int site_body_id = impl_->model->site_bodyid[impl_->site_id];
  const int body_exclude = site_body_id > 0 ? site_body_id : -1;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> points(3U * ray_count, nan);

  constexpr std::size_t kRayBatchSize = 65536;
  const std::size_t scratch_count = std::min(ray_count, kRayBatchSize);
  std::vector<mjtNum> local_directions(3U * scratch_count);
  std::vector<mjtNum> world_directions(3U * scratch_count);
  std::vector<mjtNum> distances(scratch_count, -1);
  std::vector<int> geom_ids(scratch_count, -1);

  for (std::size_t start = 0; start < ray_count; start += kRayBatchSize) {
    const std::size_t count = std::min(kRayBatchSize, ray_count - start);
    for (std::size_t offset = 0; offset < count; ++offset) {
      const std::size_t index = start + offset;
      const std::size_t ring = index / width;
      const std::size_t column = index % width;
      const double elevation = impl_->vertical_angles[ring];
      const double azimuth = kTwoPi * static_cast<double>(column) / static_cast<double>(width);
      const mjtNum local[3] = {std::cos(elevation) * std::cos(azimuth),
                               std::cos(elevation) * std::sin(azimuth), std::sin(elevation)};
      for (int axis = 0; axis < 3; ++axis) {
        local_directions[3 * offset + axis] = local[axis];
        world_directions[3 * offset + axis] = site_rotation[3 * axis] * local[0] +
                                              site_rotation[3 * axis + 1] * local[1] +
                                              site_rotation[3 * axis + 2] * local[2];
      }
    }

    mj_multiRay(impl_->model, snapshot.data(), site_position, world_directions.data(), nullptr, 1,
                body_exclude, geom_ids.data(), distances.data(), nullptr, static_cast<int>(count),
                impl_->profile.max_range_m);
    for (std::size_t offset = 0; offset < count; ++offset) {
      const double distance = distances[offset];
      if (!std::isfinite(distance) || distance < impl_->profile.min_range_m ||
          distance > impl_->profile.max_range_m || geom_ids[offset] < 0) {
        continue;
      }
      const std::size_t index = start + offset;
      points[3 * index] = static_cast<float>(distance * local_directions[3 * offset]);
      points[3 * index + 1] = static_cast<float>(distance * local_directions[3 * offset + 1]);
      points[3 * index + 2] = static_cast<float>(distance * local_directions[3 * offset + 2]);
    }
  }

  std::vector<std::byte> payload = detail::ToBytes<float>(std::span<const float>(points));
  if (payload.size() != impl_->descriptor.maximum_payload_bytes()) {
    return detail::Fail("ray lidar payload shape changed unexpectedly", error_message);
  }
  *frame = detail::MakeRawFrame(impl_->descriptor, metadata, std::move(payload));
  return true;
}

}  // namespace mujoco_sim::sensor
