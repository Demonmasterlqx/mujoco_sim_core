#pragma once
#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>
#include <mujoco_sim/adapter_sdk/adapter_domain.hpp>
namespace mujoco_sim::adapter_sdk {
struct ControllerOptions {
  std::string server_name = "default", adapter_id;
  std::vector<std::string> observation_names, command_names;
  std::uint32_t service_timeout_ms = 5000;
};
class StateLease {
 public:
  struct Data;
  StateLease();
  ~StateLease();
  StateLease(const StateLease&) = delete;
  StateLease& operator=(const StateLease&) = delete;
  StateLease(StateLease&&) noexcept;
  StateLease& operator=(StateLease&&) noexcept;
  std::uint64_t sequence() const noexcept;
  std::uint64_t tick_id() const noexcept;
  double simulation_time_s() const noexcept;
  std::span<const double> observations() const noexcept;
  explicit operator bool() const noexcept;

 private:
  std::shared_ptr<const Data> data_;
  friend class ControllerClient;
};
enum class StateReadResult {
  kReceived,
  kTimeout,
  kStopped,
  kError
};
class ControllerClient {
 public:
  explicit ControllerClient(ControllerOptions);
  ~ControllerClient();
  ControllerClient(const ControllerClient&) = delete;
  ControllerClient& operator=(const ControllerClient&) = delete;
  bool Start(std::string*);
  bool Activate(std::string*);
  StateReadResult ReadLatestState(StateLease*, std::string*);
  StateReadResult WaitForNewState(std::chrono::milliseconds, StateLease*, std::string*);
  bool SetCommand(std::span<const double>, std::string*);
  bool ReportFault(FaultCode, std::string_view, std::string*);
  void Stop();
  bool Unregister(std::string*);
  const AdapterRegistration& registration() const;
  const ControllerLayout& layout() const;

 private:
  class Impl;
  std::shared_ptr<Impl> impl_;
  friend class ControllerClientTestPeer;
};
}  // namespace mujoco_sim::adapter_sdk
