#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <limits>
#include <shared_mutex>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <mujoco_sim/protocol/data_frame.hpp>
#include <mujoco_sim_hardware_interface/mujoco_sim_system.hpp>
#include <rclcpp/rclcpp.hpp>

#include "internal/adapter_transport.hpp"
#include "internal/controller_client_test_peer.hpp"
#include "resource.hpp"

namespace mujoco_sim::hardware::internal {

/** @brief Grants deterministic tests narrow access to the directly owned SDK client.
 *         为确定性测试提供对直接持有 SDK client 的窄范围访问。 */
class MujocoSimSystemTestPeer {
 public:
  /** @brief Binds a preconfigured SDK client and opens the production I/O gate.
   *         绑定预配置 SDK client 并打开生产 I/O 门闩。 */
  static bool BindAndActivate(MujocoSimSystem& system,
                              std::unique_ptr<adapter_sdk::ControllerClient> client,
                              std::string& error) {
    if (!ValidateLayout(client->layout(), system.resources_, error)) return false;
    for (auto& resource : system.resources_) {
      if (!resource->Bind(client->layout(), error)) return false;
    }
    system.dense_command_.assign(client->layout().command_scalar_count(), 0.0);
    system.client_ = std::move(client);
    if (!system.InitializeBoundCommands(error)) {
      system.client_.reset();
      return false;
    }
    system.last_sequence_ = 0;
    system.fault_reported_ = false;
    system.active_ = true;
    return true;
  }

  /** @brief Reports whether the plugin retains a client after unregister.
   *         返回插件在注销后是否仍保留 SDK client。 */
  static bool HasClient(const MujocoSimSystem& system) { return system.client_ != nullptr; }

  /** @brief Stops only the SDK client while leaving the plugin gate open for rejection tests.
   *         仅停止 SDK client，同时为拒绝路径测试保持插件门闩开启。 */
  static void StopClient(MujocoSimSystem& system) { system.client_->Stop(); }
};

}  // namespace mujoco_sim::hardware::internal

namespace {

using namespace std::chrono_literals;
using hardware_interface::CallbackReturn;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareComponentParams;
using hardware_interface::HardwareInfo;
using mujoco_sim::adapter_sdk::ControllerClient;
using mujoco_sim::adapter_sdk::ControllerClientTestPeer;
using mujoco_sim::adapter_sdk::ControllerLayout;
using mujoco_sim::adapter_sdk::ControllerOptions;
using mujoco_sim::adapter_sdk::internal::PublishResult;
using mujoco_sim::hardware::MujocoSimSystem;
using mujoco_sim::hardware::internal::MujocoSimSystemTestPeer;

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

// AdapterTransport owns these output-pointer signatures; this mock must override them exactly.
// AdapterTransport 固定使用这些输出指针签名，本 mock 必须精确覆盖。
class FakeControllerTransport final : public mujoco_sim::adapter_sdk::internal::AdapterTransport {
 public:
  bool Register(const mujoco_sim::v4::RegisterAdapterRequest&,
                mujoco_sim::v4::RegisterAdapterResponse*, std::string*) override {
    return false;
  }

  bool Activate(const mujoco_sim::v4::ActivateAdapterRequest&,
                mujoco_sim::v4::ActivateAdapterResponse*, std::string*) override {
    return false;
  }

  bool Unregister(const mujoco_sim::v4::UnregisterAdapterRequest&,
                  mujoco_sim::v4::UnregisterAdapterResponse* response,
                  std::string* error) override {
    std::lock_guard lock(mutex_);
    ++unregister_calls_;
    if (unregister_calls_ <= failed_unregister_calls_) {
      *error = "injected unregister failure";
      return false;
    }
    response->set_code(mujoco_sim::v4::OK);
    return true;
  }

  PublishResult PublishCommand(std::string_view encoded_command) override {
    {
      std::lock_guard lock(mutex_);
      commands_.emplace_back(encoded_command);
    }
    condition_.notify_all();
    return {true, {}};
  }

  PublishResult PublishFault(const mujoco_sim::v4::ControllerFault& fault) override {
    std::lock_guard lock(mutex_);
    fault_codes_.push_back(fault.code());
    return {true, {}};
  }

  PublishResult PublishHeartbeat(const mujoco_sim::v4::SessionHeartbeat&) override {
    return {true, {}};
  }

  void FailUnregisterCalls(std::size_t count) {
    std::lock_guard lock(mutex_);
    failed_unregister_calls_ = count;
  }

  bool WaitForCommands(std::size_t count, std::chrono::milliseconds timeout) {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [&] { return commands_.size() >= count; });
  }

  std::vector<std::string> commands() const {
    std::lock_guard lock(mutex_);
    return commands_;
  }

  std::vector<mujoco_sim::v4::FaultCode> fault_codes() const {
    std::lock_guard lock(mutex_);
    return fault_codes_;
  }

  std::size_t unregister_calls() const {
    std::lock_guard lock(mutex_);
    return unregister_calls_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<std::string> commands_;
  std::vector<mujoco_sim::v4::FaultCode> fault_codes_;
  std::size_t failed_unregister_calls_ = 0;
  std::size_t unregister_calls_ = 0;
};

HardwareInfo TestHardwareInfo() {
  HardwareInfo info;
  info.name = "system_io_test";
  info.type = "system";
  info.rw_rate = 100;
  info.is_async = false;
  info.thread_priority = 50;
  info.hardware_plugin_name = "mujoco_sim_hardware_interface/MujocoSimSystem";
  info.hardware_parameters = {{"adapter_id", "system_io_controller"}};
  ComponentInfo joint;
  joint.name = "wheel";
  joint.type = "joint";
  joint.parameters = {{"actuator_velocity", "wheel_actuator"},
                      {"sensor_position", "wheel_position"},
                      {"sensor_velocity", "wheel_velocity"}};
  info.joints.push_back(std::move(joint));
  return info;
}

HardwareComponentParams Params(HardwareInfo info) {
  HardwareComponentParams params;
  params.hardware_info = std::move(info);
  params.clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  params.logger = rclcpp::get_logger("mujoco_sim_hardware_interface_io_test");
  return params;
}

ControllerLayout TestLayout() {
  return ControllerLayout(
      {{"wheel_velocity", {0, 1}, false, 0.0, 0.0}, {"wheel_position", {1, 1}, false, 0.0, 0.0}},
      {{"wheel_actuator", {0, 1}, true, -2.0, 2.0}}, 2, 1);
}

std::unique_ptr<ControllerClient> TestClient(
    const std::shared_ptr<FakeControllerTransport>& transport) {
  ControllerOptions options;
  options.adapter_id = "system_io_controller";
  options.observation_names = {"wheel_position", "wheel_velocity"};
  options.command_names = {"wheel_actuator"};
  auto client = std::make_unique<ControllerClient>(std::move(options));
  ControllerClientTestPeer::ConfigureWithLayout(*client, transport, TestLayout(), 20ms);
  ControllerClientTestPeer::EnableRemoteRegistration(*client);
  return client;
}

void PrepareSystem(MujocoSimSystem& system,
                   std::vector<hardware_interface::StateInterface::ConstSharedPtr>& states,
                   std::vector<hardware_interface::CommandInterface::SharedPtr>& commands) {
  ASSERT_EQ(system.init(Params(TestHardwareInfo())), CallbackReturn::SUCCESS);
  states = system.on_export_state_interfaces();
  commands = system.on_export_command_interfaces();
  ASSERT_EQ(states.size(), 2U);
  ASSERT_EQ(commands.size(), 1U);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

void SetCommand(const hardware_interface::CommandInterface::SharedPtr& handle, double value) {
  std::unique_lock<std::shared_mutex> lock(handle->get_mutex());
  ASSERT_TRUE(handle->set_value(lock, value));
}

std::vector<double> DecodeCommand(const std::string& encoded) {
  mujoco_sim::protocol::ParsedDataFrame parsed;
  mujoco_sim::protocol::DataFrameError error;
  EXPECT_TRUE(mujoco_sim::protocol::ParseDataFrame(encoded, &parsed, &error)) << error.message;
  std::vector<double> values(parsed.scalar_count);
  EXPECT_TRUE(mujoco_sim::protocol::DecodeDataFrameScalars(parsed, values, &error))
      << error.message;
  return values;
}

TEST(SystemIoTest, AppliesOnlyCompleteFiniteStateFramesAndReportsOneFault) {
  MujocoSimSystem system;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> states;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  PrepareSystem(system, states, commands);
  auto transport = std::make_shared<FakeControllerTransport>();
  auto client = TestClient(transport);
  auto& client_reference = *client;
  std::string error;
  ASSERT_TRUE(MujocoSimSystemTestPeer::BindAndActivate(system, std::move(client), error)) << error;

  const std::array first_frame{3.0, 1.5};
  ControllerClientTestPeer::HandleState(client_reference, 1, 1, first_frame);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(states[0]->get_optional<double>().value(), 1.5);
  EXPECT_DOUBLE_EQ(states[1]->get_optional<double>().value(), 3.0);

  const std::array invalid_frame{std::numeric_limits<double>::quiet_NaN(), 9.0};
  ControllerClientTestPeer::HandleState(client_reference, 2, 2, invalid_frame);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(states[0]->get_optional<double>().value(), 1.5);
  EXPECT_DOUBLE_EQ(states[1]->get_optional<double>().value(), 3.0);
  ASSERT_EQ(transport->fault_codes().size(), 1U);
  EXPECT_EQ(transport->fault_codes()[0], mujoco_sim::v4::INTERNAL_CONTROLLER_ERROR);

  const std::array recovered_frame{4.0, 2.5};
  ControllerClientTestPeer::HandleState(client_reference, 3, 3, recovered_frame);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(states[0]->get_optional<double>().value(), 2.5);
  EXPECT_DOUBLE_EQ(states[1]->get_optional<double>().value(), 4.0);
  EXPECT_EQ(system.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(SystemIoTest, ClampsAndSanitizesDenseCommandAndSurfacesSdkRejection) {
  MujocoSimSystem system;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> states;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  PrepareSystem(system, states, commands);
  auto transport = std::make_shared<FakeControllerTransport>();
  auto client = TestClient(transport);
  auto& client_reference = *client;
  std::string error;
  ASSERT_TRUE(MujocoSimSystemTestPeer::BindAndActivate(system, std::move(client), error)) << error;

  SetCommand(commands[0], 8.0);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(commands[0]->get_optional<double>().value(), 2.0);
  ControllerClientTestPeer::StartPublisher(client_reference);
  ASSERT_TRUE(transport->WaitForCommands(1, 1s));
  const auto dense = DecodeCommand(transport->commands().front());
  ASSERT_EQ(dense.size(), 1U);
  EXPECT_DOUBLE_EQ(dense[0], 2.0);

  SetCommand(commands[0], std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(commands[0]->get_optional<double>().value(), 0.0);
  ASSERT_TRUE(transport->WaitForCommands(2, 1s));
  const auto sanitized_dense = DecodeCommand(transport->commands().back());
  ASSERT_EQ(sanitized_dense.size(), 1U);
  EXPECT_DOUBLE_EQ(sanitized_dense[0], 0.0);
  EXPECT_TRUE(transport->fault_codes().empty());

  SetCommand(commands[0], 1.0);
  MujocoSimSystemTestPeer::StopClient(system);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(SystemIoTest, FailedUnregisterStopsPublisherRetainsClientAndRetriesCleanup) {
  MujocoSimSystem system;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> states;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  PrepareSystem(system, states, commands);
  auto transport = std::make_shared<FakeControllerTransport>();
  transport->FailUnregisterCalls(1);
  auto client = TestClient(transport);
  auto& client_reference = *client;
  std::string error;
  ASSERT_TRUE(MujocoSimSystemTestPeer::BindAndActivate(system, std::move(client), error)) << error;
  SetCommand(commands[0], 1.0);
  ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  ControllerClientTestPeer::StartPublisher(client_reference);
  ASSERT_TRUE(transport->WaitForCommands(1, 1s));

  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  ASSERT_TRUE(MujocoSimSystemTestPeer::HasClient(system));
  const std::size_t stopped_count = transport->commands().size();
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(transport->commands().size(), stopped_count);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);

  EXPECT_EQ(system.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_FALSE(MujocoSimSystemTestPeer::HasClient(system));
  EXPECT_EQ(transport->unregister_calls(), 2U);
}

}  // namespace
