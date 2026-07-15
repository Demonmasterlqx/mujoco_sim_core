#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

#include <Eigen/Geometry>

#include <mujoco_sim/adapter_sdk/observer_client.hpp>
#include <mujoco_sim/adapter_sdk/sensor_stream_client.hpp>
#include <mujoco_sim/sensor_demo/imu_synchronizer.hpp>
#include <mujoco_sim/sensor_demo/interface_data.hpp>
#include <mujoco_sim/sensor_demo/map_output.hpp>
#include <mujoco_sim/sensor_demo/stream_processing.hpp>

#include "app_config.hpp"
#include "mapper_scan_accounting.hpp"
#include "process_signal.hpp"

namespace demo = mujoco_sim::sensor_demo;

namespace {

bool AddImuFrame(const mujoco_sim::adapter_sdk::ObservationLease& frame,
                 const demo::ImuOrientationDecoder& decoder,
                 demo::ImuSynchronizer* synchronizer,
                 std::string* error_message) {
  demo::ImuOrientationSample sample;
  if (!decoder.Decode(frame, &sample, error_message)) {
    return false;
  }
  demo::TimedOrientation timed;
  timed.simulation_time = sample.simulation_time;
  timed.orientation = Eigen::Quaterniond(
      sample.quaternion[0], sample.quaternion[1], sample.quaternion[2],
      sample.quaternion[3]);
  return synchronizer->Add(std::move(timed), error_message);
}

}  // namespace

int main(int argc, char** argv) {
  demo::AppConfig app_config;
  std::string message;
  const auto parse_result = demo::ParseAppConfig(argc, argv, &app_config,
                                                  &message);
  if (parse_result == demo::ParseAppConfigResult::kHelp) {
    std::cout << message;
    return 0;
  }
  if (parse_result != demo::ParseAppConfigResult::kSuccess) {
    std::cerr << message << '\n';
    return 1;
  }
  demo::InstallProcessSignalHandlers();
  const auto& config = app_config.mapper;

  mujoco_sim::adapter_sdk::ObserverClient observer(config.observer);
  if (!observer.Start(&message) || !observer.Activate(&message)) {
    std::cerr << "mapper observer start failed: " << message << '\n';
    std::string ignored;
    observer.Unregister(&ignored);
    return 1;
  }
  mujoco_sim::adapter_sdk::SensorStreamClient streams(config.streams);
  if (!streams.Start(&message)) {
    std::cerr << "mapper LiDAR stream start failed: " << message << '\n';
    std::string ignored;
    observer.Unregister(&ignored);
    return 1;
  }
  const auto lidar_descriptor = streams.descriptor(config.lidar_stream);
  if (!lidar_descriptor.has_value()) {
    std::cerr << "mapper LiDAR descriptor is missing\n";
    std::string ignored;
    observer.Unregister(&ignored);
    return 1;
  }

  demo::ImuSynchronizer imu(config.observer.queue_capacity);
  const demo::ImuOrientationDecoder imu_decoder(observer.layout(),
                                                 config.imu_quaternion);
  demo::SimpleIcpMapper mapper(config.algorithm);
  std::cout << "sensor-demo mapper ready observer="
            << config.observer.observer_id
            << " stream=" << config.lidar_stream << std::endl;

  demo::MapperScanAccounting scan_accounting(config.max_scans);
  int result = 0;
  try {
    while (!demo::ProcessStopRequested() && scan_accounting.CanStartScan()) {
      while (true) {
        mujoco_sim::adapter_sdk::ObservationLease frame;
        const auto observation_result = observer.WaitForObservation(
            std::chrono::milliseconds(0), &frame, &message);
        if (observation_result ==
            mujoco_sim::adapter_sdk::ObservationWaitResult::kTimeout) {
          break;
        }
        if (observation_result !=
            mujoco_sim::adapter_sdk::ObservationWaitResult::kReceived) {
          std::cerr << "mapper IMU stream stopped: " << message << '\n';
          result = 1;
          break;
        }
        if (!AddImuFrame(frame, imu_decoder, &imu, &message)) {
          std::cerr << "mapper IMU validation failed: " << message << '\n';
          result = 1;
          break;
        }
      }
      if (result != 0) break;

      mujoco_sim::adapter_sdk::OwnedStreamSample sample;
      const auto stream_result = streams.WaitForSample(
          config.lidar_stream,
          std::chrono::milliseconds(config.wait_timeout_ms), &sample,
          &message);
      if (stream_result ==
          mujoco_sim::adapter_sdk::SampleWaitResult::kTimeout) {
        continue;
      }
      if (stream_result !=
          mujoco_sim::adapter_sdk::SampleWaitResult::kReceived) {
        std::cerr << "mapper LiDAR stream stopped: " << message << '\n';
        result = 1;
        break;
      }
      demo::PointCloud cloud;
      if (!demo::DecodeXyzPointCloud(sample, *lidar_descriptor, &cloud,
                                     &message)) {
        std::cerr << "mapper LiDAR validation failed: " << message << '\n';
        result = 1;
        break;
      }
      demo::MapperScanScope scan(scan_accounting);
      const std::uint64_t scan_number = scan.scan_number();
      const double timestamp = sample.header.simulation_time_s();
      auto orientation = imu.FindNearest(timestamp, config.imu_max_age_s);
      const auto imu_deadline =
          std::chrono::steady_clock::now() +
          std::chrono::milliseconds(config.wait_timeout_ms);
      while (!orientation.has_value() && !demo::ProcessStopRequested() &&
             std::chrono::steady_clock::now() < imu_deadline) {
        mujoco_sim::adapter_sdk::ObservationLease frame;
        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                imu_deadline - std::chrono::steady_clock::now());
        const auto observation_result = observer.WaitForObservation(
            std::max(std::chrono::milliseconds(0), remaining), &frame,
            &message);
        if (observation_result ==
            mujoco_sim::adapter_sdk::ObservationWaitResult::kTimeout) {
          break;
        }
        if (observation_result !=
                mujoco_sim::adapter_sdk::ObservationWaitResult::kReceived ||
            !AddImuFrame(frame, imu_decoder, &imu, &message)) {
          std::cerr << "mapper IMU synchronization failed: " << message
                    << '\n';
          result = 1;
          break;
        }
        orientation = imu.FindNearest(timestamp, config.imu_max_age_s);
      }
      if (result != 0) break;
      if (!orientation.has_value()) continue;
      try {
        const demo::IcpUpdateResult update =
            mapper.AddScan(timestamp, cloud, orientation->orientation);
        scan.Finish(update.accepted ? demo::MapperScanOutcome::kAccepted
                                    : demo::MapperScanOutcome::kRejected);
        if (scan_number % 20 == 0) {
          std::cout << "sensor-demo mapper scans=" << scan_number
                    << " accepted=" << scan_accounting.accepted_scans()
                    << " points=" << update.global_map_points
                    << " fitness=" << update.fitness << '\n';
        }
      } catch (const demo::IcpMapperError& error) {
        std::cerr << "mapper rejected scan " << scan_number << ": "
                  << error.what() << '\n';
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "mapper processing failed: " << error.what() << '\n';
    result = 1;
  }

  streams.Stop();
  std::string unregister_error;
  if (!observer.Unregister(&unregister_error)) {
    std::cerr << "mapper observer unregister failed: " << unregister_error
              << '\n';
    if (result == 0 && !demo::ProcessStopRequested()) result = 1;
  }
  if (mapper.scan_count() > 0) {
    try {
      const auto paths = demo::SaveMapOutputs(
          config.output_directory, mapper, config.output);
      std::cout << "sensor-demo mapper outputs pcd=" << paths.pcd
                << " trajectory=" << paths.trajectory_csv
                << " png=" << paths.top_down_png << '\n';
    } catch (const std::exception& error) {
      std::cerr << "mapper output failed: " << error.what() << '\n';
      result = 1;
    }
  } else {
    std::cerr << "mapper stopped before accepting any LiDAR scan\n";
    result = 1;
  }
  std::cout << "sensor-demo mapper summary processed="
            << scan_accounting.processed_scans()
            << " accepted=" << scan_accounting.accepted_scans()
            << " rejected=" << scan_accounting.rejected_scans()
            << " dropped=" << streams.dropped_samples(config.lidar_stream)
            << '\n';
  return result;
}
