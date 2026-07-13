#include "app_config.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mujoco_sim::sensor_demo {
namespace {

bool Fail(const std::string& message, std::string* error_message) {
  if (error_message != nullptr) *error_message = message;
  return false;
}

template <std::size_t Size>
std::array<std::string, Size> CopyNames(
    const std::array<std::string, Size>& values) {
  return values;
}

bool RequireUnique(const std::vector<std::string>& values,
                   const std::string& group, std::string* error_message) {
  std::unordered_set<std::string> names;
  for (const auto& value : values) {
    if (value.empty()) {
      return Fail(group + " contains an empty name", error_message);
    }
    if (!names.insert(value).second) {
      return Fail(group + " contains a duplicate name: " + value,
                  error_message);
    }
  }
  return true;
}

bool ValidateObserverOptions(
    const mujoco_sim::adapter_sdk::ObserverOptions& options,
    const std::string& label, std::string* error_message) {
  if (options.timeout_ms != 0 &&
      options.keepalive_period_ms >= options.timeout_ms) {
    return Fail(label +
                    ".keepalive_period_ms must be less than timeout_ms",
                error_message);
  }
  return RequireUnique(options.observation_names, label + " observations",
                       error_message);
}

bool IsLeafFilename(const std::string& value) {
  const std::filesystem::path path(value);
  return !value.empty() && !path.is_absolute() && !path.has_parent_path() &&
         path.filename() == path && path != "." && path != "..";
}

ControllerInterfaces BuildControllerInterfaces(
    const generated::SensorDemoConfig::Controller::Interfaces& source) {
  ControllerInterfaces result;
  result.wheel_positions = CopyNames(source.wheel_positions);
  result.imu_quaternion = source.imu_quaternion;
  result.imu_angular_velocity = source.imu_angular_velocity;
  result.wheel_velocity_commands =
      CopyNames(source.wheel_velocity_commands);
  return result;
}

StateObserverInterfaces BuildStateInterfaces(
    const generated::SensorDemoConfig::StateObserver::Interfaces& source) {
  StateObserverInterfaces result;
  result.wheel_positions = CopyNames(source.wheel_positions);
  result.wheel_velocities = CopyNames(source.wheel_velocities);
  result.imu_quaternion = source.imu_quaternion;
  result.imu_angular_velocity = source.imu_angular_velocity;
  result.imu_linear_acceleration = source.imu_linear_acceleration;
  return result;
}

WaypointControllerConfig BuildControllerConfig(
    const generated::SensorDemoConfig::Controller::Control& source) {
  WaypointControllerConfig result;
  result.wheel_radius = source.wheel_radius;
  result.track_width = source.track_width;
  result.target_speed = source.target_speed;
  result.max_wheel_angular_speed = source.max_wheel_angular_speed;
  result.heading_gain = source.heading_gain;
  result.max_yaw_rate = source.max_yaw_rate;
  result.waypoint_tolerance = source.waypoint_tolerance;
  result.corner_slowdown_distance = source.corner_slowdown_distance;
  result.stop_after_lap = source.stop_after_lap;
  result.initial_pose = {source.initial_pose.x, source.initial_pose.y,
                         source.initial_pose.yaw};
  result.waypoints.clear();
  result.waypoints.reserve(source.waypoints.size());
  for (const auto& waypoint : source.waypoints) {
    result.waypoints.push_back({waypoint.x, waypoint.y});
  }
  return result;
}

}  // namespace

bool BuildAppConfig(const generated::SensorDemoConfig& generated_config,
                    AppConfig* config, std::string* error_message) {
  if (config == nullptr || error_message == nullptr) {
    return Fail("config and error_message must not be null", error_message);
  }
  error_message->clear();
  AppConfig next;
  next.server_name = generated_config.common.server_name;
  next.output_directory = generated_config.common.output_directory;
  if (next.server_name.empty() || next.output_directory.empty()) {
    return Fail("server name and output directory must not be empty",
                error_message);
  }

  next.controller.interfaces =
      BuildControllerInterfaces(generated_config.controller.interfaces);
  next.controller.adapter.server_name = next.server_name;
  next.controller.adapter.adapter_id = generated_config.controller.adapter_id;
  next.controller.adapter.observation_names =
      ControllerObservationNames(next.controller.interfaces);
  next.controller.adapter.command_names =
      ControllerCommandNames(next.controller.interfaces);
  next.controller.adapter.timeout_ms = generated_config.controller.timeout_ms;
  next.controller.adapter.service_timeout_ms =
      generated_config.common.service_timeout_ms;
  next.controller.control =
      BuildControllerConfig(generated_config.controller.control);
  next.controller.log_every_ticks =
      generated_config.controller.log_every_ticks;
  next.controller.max_simulation_time_s =
      generated_config.controller.max_simulation_time_s;
  next.controller.wait_timeout_ms = generated_config.common.wait_timeout_ms;

  next.state_observer.interfaces =
      BuildStateInterfaces(generated_config.state_observer.interfaces);
  next.state_observer.observer.server_name = next.server_name;
  next.state_observer.observer.observer_id =
      generated_config.state_observer.adapter_id;
  next.state_observer.observer.observation_names =
      StateObservationNames(next.state_observer.interfaces);
  next.state_observer.observer.timeout_ms =
      generated_config.state_observer.timeout_ms;
  next.state_observer.observer.service_timeout_ms =
      generated_config.common.service_timeout_ms;
  next.state_observer.observer.keepalive_period_ms =
      generated_config.state_observer.keepalive_period_ms;
  next.state_observer.observer.every_n_steps =
      generated_config.state_observer.every_n_steps;
  next.state_observer.observer.period_s.reset();
  next.state_observer.observer.queue_capacity =
      generated_config.state_observer.queue_capacity;
  next.state_observer.log_every_frames =
      generated_config.state_observer.log_every_frames;
  next.state_observer.max_frames = generated_config.state_observer.max_frames;
  next.state_observer.wait_timeout_ms = generated_config.common.wait_timeout_ms;

  const auto& camera = generated_config.camera_consumer;
  next.camera_consumer.rgb_stream = camera.streams.rgb;
  next.camera_consumer.depth_stream = camera.streams.depth;
  next.camera_consumer.segmentation_stream = camera.streams.segmentation;
  next.camera_consumer.pointcloud_stream = camera.streams.pointcloud;
  next.camera_consumer.streams.server_name = next.server_name;
  next.camera_consumer.streams.stream_names = {
      next.camera_consumer.rgb_stream, next.camera_consumer.depth_stream,
      next.camera_consumer.segmentation_stream,
      next.camera_consumer.pointcloud_stream};
  next.camera_consumer.streams.service_timeout_ms =
      generated_config.common.service_timeout_ms;
  next.camera_consumer.streams.queue_capacity = camera.queue_capacity;
  next.camera_consumer.screenshot_path =
      next.output_directory / camera.screenshot_filename;
  next.camera_consumer.max_samples_per_stream =
      camera.max_samples_per_stream;
  next.camera_consumer.wait_timeout_ms = generated_config.common.wait_timeout_ms;

  const auto& mapper = generated_config.mapper;
  next.mapper.observer.server_name = next.server_name;
  next.mapper.observer.observer_id = mapper.adapter_id;
  next.mapper.observer.observation_names = {mapper.imu_quaternion};
  next.mapper.observer.timeout_ms = mapper.timeout_ms;
  next.mapper.observer.service_timeout_ms =
      generated_config.common.service_timeout_ms;
  next.mapper.observer.keepalive_period_ms = mapper.keepalive_period_ms;
  next.mapper.observer.every_n_steps = mapper.every_n_steps;
  next.mapper.observer.period_s.reset();
  next.mapper.observer.queue_capacity = mapper.observer_queue_capacity;
  next.mapper.streams.server_name = next.server_name;
  next.mapper.streams.stream_names = {mapper.lidar_stream};
  next.mapper.streams.service_timeout_ms =
      generated_config.common.service_timeout_ms;
  next.mapper.streams.queue_capacity = mapper.stream_queue_capacity;
  next.mapper.imu_quaternion = mapper.imu_quaternion;
  next.mapper.lidar_stream = mapper.lidar_stream;
  next.mapper.output_directory = next.output_directory;
  next.mapper.imu_max_age_s = mapper.imu_max_age_s;
  next.mapper.max_scans = mapper.max_scans;
  next.mapper.wait_timeout_ms = generated_config.common.wait_timeout_ms;
  next.mapper.algorithm.voxel_size = mapper.algorithm.voxel_size;
  next.mapper.algorithm.max_correspondence_distance =
      mapper.algorithm.max_correspondence_distance;
  next.mapper.algorithm.max_iterations =
      static_cast<int>(mapper.algorithm.max_iterations);
  next.mapper.algorithm.local_map_scans = mapper.algorithm.local_map_scans;
  next.mapper.algorithm.minimum_points = mapper.algorithm.minimum_points;
  next.mapper.algorithm.transformation_epsilon =
      mapper.algorithm.transformation_epsilon;
  next.mapper.algorithm.euclidean_fitness_epsilon =
      mapper.algorithm.euclidean_fitness_epsilon;
  next.mapper.algorithm.maximum_fitness_score =
      mapper.algorithm.maximum_fitness_score;
  next.mapper.algorithm.minimum_inlier_ratio =
      mapper.algorithm.minimum_inlier_ratio;
  next.mapper.algorithm.maximum_translation_step_m =
      mapper.algorithm.maximum_translation_step_m;
  next.mapper.algorithm.maximum_rotation_step_rad =
      mapper.algorithm.maximum_rotation_step_rad;
  next.mapper.output.pcd_filename = mapper.output.pcd_filename;
  next.mapper.output.trajectory_filename = mapper.output.trajectory_filename;
  next.mapper.output.top_down_png_filename =
      mapper.output.top_down_png_filename;
  next.mapper.output.image_width = mapper.output.image_width;
  next.mapper.output.image_height = mapper.output.image_height;
  next.mapper.output.padding_meters = mapper.output.padding_meters;

  if (!RequireUnique(next.controller.adapter.observation_names,
                     "controller observations", error_message) ||
      !RequireUnique(next.controller.adapter.command_names,
                     "controller commands", error_message) ||
      !ValidateObserverOptions(next.state_observer.observer,
                               "state_observer", error_message) ||
      !ValidateObserverOptions(next.mapper.observer, "mapper",
                               error_message) ||
      !RequireUnique(next.camera_consumer.streams.stream_names,
                     "camera streams", error_message)) {
    return false;
  }
  std::vector<std::string> adapter_ids = {
      next.controller.adapter.adapter_id,
      next.state_observer.observer.observer_id,
      next.mapper.observer.observer_id};
  if (!RequireUnique(adapter_ids, "adapter IDs", error_message)) return false;
  std::vector<std::string> all_streams =
      next.camera_consumer.streams.stream_names;
  all_streams.push_back(next.mapper.lidar_stream);
  if (!RequireUnique(all_streams, "sensor streams", error_message)) {
    return false;
  }

  const std::array<std::string, 4> output_filenames = {
      generated_config.camera_consumer.screenshot_filename,
      next.mapper.output.pcd_filename, next.mapper.output.trajectory_filename,
      next.mapper.output.top_down_png_filename};
  std::vector<std::string> output_names(output_filenames.begin(),
                                        output_filenames.end());
  if (!std::all_of(output_names.begin(), output_names.end(), IsLeafFilename)) {
    return Fail("all output filenames must be leaf filenames", error_message);
  }
  if (!RequireUnique(output_names, "output filenames", error_message)) {
    return false;
  }

  try {
    const WaypointController validation(next.controller.control);
    const SimpleIcpMapper mapper_validation(next.mapper.algorithm);
    (void)validation;
    (void)mapper_validation;
  } catch (const std::exception& error) {
    return Fail(std::string("invalid algorithm configuration: ") + error.what(),
                error_message);
  }

  *config = std::move(next);
  return true;
}

ParseAppConfigResult ParseAppConfig(int argc, char** argv, AppConfig* config,
                                    std::string* message) {
  if (config == nullptr || message == nullptr) {
    if (message != nullptr) *message = "config and message must not be null";
    return ParseAppConfigResult::kError;
  }
  std::vector<const char*> arguments;
  arguments.reserve(static_cast<std::size_t>(std::max(argc, 0)));
  for (int i = 0; i < argc; ++i) arguments.push_back(argv[i]);

  generated::SensorDemoConfig generated_config;
  std::string parse_message;
  if (!generated::Parse(argc, arguments.data(), &generated_config,
                        &parse_message)) {
    *message = std::move(parse_message);
    return *message == generated::Usage() ? ParseAppConfigResult::kHelp
                                          : ParseAppConfigResult::kError;
  }
  AppConfig next;
  if (!BuildAppConfig(generated_config, &next, &parse_message)) {
    *message = std::move(parse_message);
    return ParseAppConfigResult::kError;
  }
  *config = std::move(next);
  message->clear();
  return ParseAppConfigResult::kSuccess;
}

std::string AppUsage() { return generated::Usage(); }

}  // namespace mujoco_sim::sensor_demo
