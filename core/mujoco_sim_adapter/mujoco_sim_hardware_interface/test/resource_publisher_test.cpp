#include "resource.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>

namespace {

using namespace std::chrono_literals;
using mujoco_sim::adapter_sdk::ControllerLayout;
using mujoco_sim::adapter_sdk::ResourceInfo;
using mujoco_sim::hardware::internal::InterfaceRegistry;
using mujoco_sim::hardware::internal::MakeResource;
using mujoco_sim::hardware::internal::ParameterMap;
using mujoco_sim::hardware::internal::ResourceInterface;

class RosEnvironment final : public testing::Environment {
 public:
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
};

[[maybe_unused]] const auto* kRosEnvironment =
    testing::AddGlobalTestEnvironment(new RosEnvironment);

hardware_interface::ComponentInfo Component(std::string name, std::string type) {
  hardware_interface::ComponentInfo component;
  component.name = std::move(name);
  component.type = std::move(type);
  return component;
}

ResourceInfo Observation(std::string name, std::size_t offset, std::size_t count) {
  return {std::move(name), {offset, count}, false, 0.0, 0.0};
}

ResourceInfo Command(std::string name, std::size_t offset, double minimum, double maximum) {
  return {std::move(name), {offset, 1}, true, minimum, maximum};
}

void Register(ResourceInterface& resource) {
  InterfaceRegistry registry;
  std::string error;
  ASSERT_TRUE(resource.RegisterInterfaces(registry, error)) << error;
  (void)registry.ReleaseManifest();
}

template <class Predicate>
bool SpinUntil(rclcpp::executors::SingleThreadedExecutor& executor, Predicate&& predicate,
               std::chrono::milliseconds timeout = 2s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (std::invoke(predicate)) return true;
    std::this_thread::sleep_for(5ms);
  }
  executor.spin_some();
  return std::invoke(predicate);
}

TEST(ResourcePublisherTest, JointPublishesOnlyCommittedAndAcceptedValues) {
  std::string error;
  auto resource = MakeResource(
      Component("wheel", "joint"),
      ParameterMap{
          {"sensor_position", "position"}, {"actuator_velocity", "motor"}, {"debug", "true"}},
      error);
  ASSERT_NE(resource, nullptr) << error;
  Register(*resource);
  const ControllerLayout layout({Observation("position", 0, 1)}, {Command("motor", 0, -2.0, 2.0)},
                                1, 1);
  ASSERT_TRUE(resource->Bind(layout, error)) << error;

  const auto node = std::make_shared<rclcpp::Node>("joint_resource_publisher_test");
  std::size_t state_count = 0;
  std::size_t command_count = 0;
  double state_value = 0.0;
  double command_value = 0.0;
  const auto state_subscription = node->create_subscription<std_msgs::msg::Float64>(
      "~/debug/joints/wheel/state/position", rclcpp::SensorDataQoS(),
      [&](const std_msgs::msg::Float64& message) {
        ++state_count;
        state_value = message.data;
      });
  const auto command_subscription = node->create_subscription<std_msgs::msg::Float64>(
      "~/debug/joints/wheel/command/velocity", rclcpp::SensorDataQoS(),
      [&](const std_msgs::msg::Float64& message) {
        ++command_count;
        command_value = message.data;
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(resource->ConfigurePublishers(node, error)) << error;
  ASSERT_TRUE(SpinUntil(executor, [&] {
    return state_subscription->get_publisher_count() == 1 &&
           command_subscription->get_publisher_count() == 1;
  }));

  const std::array observations{1.25};
  std::array<double, 1> state{};
  ASSERT_TRUE(resource->StageState(observations, state, error)) << error;
  std::array command{8.0};
  std::array<double, 1> dense_command{};
  ASSERT_TRUE(resource->StageCommand(command, dense_command, error)) << error;
  EXPECT_FALSE(SpinUntil(executor, [&] { return state_count != 0 || command_count != 0; }, 50ms));

  const builtin_interfaces::msg::Time stamp{};
  resource->OnStateCommitted(state, stamp);
  resource->OnCommandAccepted(command);
  ASSERT_TRUE(SpinUntil(executor, [&] { return state_count == 1 && command_count == 1; }));
  EXPECT_DOUBLE_EQ(state_value, 1.25);
  EXPECT_DOUBLE_EQ(command_value, 2.0);
}

TEST(ResourcePublisherTest, GenericSensorPublishesOneOrderedArrayAndOwnsReset) {
  std::string error;
  auto resource = MakeResource(Component("wrench", "sensor"),
                               ParameterMap{{"sensor_type", "other"},
                                            {"sensor_name", "force"},
                                            {"sensor_dim", "3"},
                                            {"sensor_dim_names", "z x y"},
                                            {"debug", "true"}},
                               error);
  ASSERT_NE(resource, nullptr) << error;
  Register(*resource);
  const ControllerLayout layout({Observation("force", 0, 3)}, {}, 3, 0);
  ASSERT_TRUE(resource->Bind(layout, error)) << error;

  const auto node = std::make_shared<rclcpp::Node>("sensor_resource_publisher_test");
  std::size_t message_count = 0;
  std_msgs::msg::Float64MultiArray last_message;
  const auto subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/debug/sensors/wrench", rclcpp::SensorDataQoS(),
      [&](const std_msgs::msg::Float64MultiArray& message) {
        ++message_count;
        last_message = message;
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(resource->ConfigurePublishers(node, error)) << error;
  ASSERT_TRUE(SpinUntil(executor, [&] { return subscription->get_publisher_count() == 1; }));

  const std::array observations{3.0, 1.0, 2.0};
  std::array<double, 3> state{};
  ASSERT_TRUE(resource->StageState(observations, state, error)) << error;
  const builtin_interfaces::msg::Time stamp{};
  resource->OnStateCommitted(state, stamp);
  ASSERT_TRUE(SpinUntil(executor, [&] { return message_count == 1; }));
  EXPECT_EQ(last_message.data, (std::vector<double>{3.0, 1.0, 2.0}));
  ASSERT_EQ(last_message.layout.dim.size(), 1U);
  EXPECT_EQ(last_message.layout.dim[0].label, "wrench");
  EXPECT_EQ(last_message.layout.dim[0].size, 3U);

  resource->ResetPublishers();
  resource->OnStateCommitted(state, stamp);
  EXPECT_FALSE(SpinUntil(executor, [&] { return message_count > 1; }, 100ms));
}

TEST(ResourcePublisherTest, ImuPublishesGroupedArrayAndItsOwnTransform) {
  std::string error;
  auto resource = MakeResource(Component("imu_sensor", "sensor"),
                               ParameterMap{{"sensor_type", "imu"},
                                            {"accelerometer", "accel"},
                                            {"gyro", "gyro"},
                                            {"target_link", "imu_link"},
                                            {"father_link", "world"},
                                            {"orientation", "quat"},
                                            {"position", "position"},
                                            {"debug", "true"}},
                               error);
  ASSERT_NE(resource, nullptr) << error;
  Register(*resource);
  const ControllerLayout layout({Observation("accel", 0, 3), Observation("gyro", 3, 3),
                                 Observation("quat", 6, 4), Observation("position", 10, 3)},
                                {}, 13, 0);
  ASSERT_TRUE(resource->Bind(layout, error)) << error;

  const auto node = std::make_shared<rclcpp::Node>("imu_resource_publisher_test");
  std::size_t debug_count = 0;
  std_msgs::msg::Float64MultiArray debug_message;
  tf2_msgs::msg::TFMessage transform_message;
  const auto debug_subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/debug/imus/imu_sensor", rclcpp::SensorDataQoS(),
      [&](const std_msgs::msg::Float64MultiArray& message) {
        ++debug_count;
        debug_message = message;
      });
  const auto transform_subscription = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", tf2_ros::DynamicListenerQoS(), [&](const tf2_msgs::msg::TFMessage& message) {
        for (const auto& transform : message.transforms) {
          if (transform.child_frame_id == "imu_link") transform_message = message;
        }
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(resource->ConfigurePublishers(node, error)) << error;
  ASSERT_TRUE(SpinUntil(executor, [&] {
    return debug_subscription->get_publisher_count() == 1 &&
           transform_subscription->get_publisher_count() >= 1;
  }));

  const std::array observations{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 2.0, 0.0, 0.0, 0.0, 7.0, 8.0, 9.0};
  std::array<double, 13> state{};
  ASSERT_TRUE(resource->StageState(observations, state, error)) << error;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 345;
  resource->OnStateCommitted(state, stamp);
  ASSERT_TRUE(SpinUntil(executor,
                        [&] { return debug_count == 1 && !transform_message.transforms.empty(); }));

  EXPECT_EQ(debug_message.data,
            (std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0, 7.0, 8.0, 9.0}));
  ASSERT_EQ(transform_message.transforms.size(), 1U);
  const auto& transform = transform_message.transforms.front();
  EXPECT_EQ(transform.header.frame_id, "world");
  EXPECT_EQ(transform.child_frame_id, "imu_link");
  EXPECT_EQ(transform.header.stamp, stamp);
  EXPECT_DOUBLE_EQ(transform.transform.rotation.w, 1.0);
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 7.0);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, 8.0);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 9.0);
}

}  // namespace
