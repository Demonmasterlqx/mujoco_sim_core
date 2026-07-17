#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace mujoco_sim::adapter_sdk {

struct ScalarSpan {
  std::size_t offset = 0;
  std::size_t count = 0;
};

struct ResourceInfo {
  std::string name;
  ScalarSpan scalars;
  bool has_command_range = false;
  double command_min = 0.0;
  double command_max = 0.0;
};

class ControllerLayout {
 public:
  ControllerLayout() = default;
  ControllerLayout(std::vector<ResourceInfo> observations, std::vector<ResourceInfo> commands,
                   std::size_t observation_scalar_count, std::size_t command_scalar_count);
  std::optional<ScalarSpan> Observation(std::string_view name) const;
  std::optional<ScalarSpan> Command(std::string_view name) const;
  std::size_t observation_scalar_count() const noexcept;
  std::size_t command_scalar_count() const noexcept;
  const std::vector<ResourceInfo>& observations() const noexcept;
  const std::vector<ResourceInfo>& commands() const noexcept;

 private:
  std::vector<ResourceInfo> observations_;
  std::vector<ResourceInfo> commands_;
  std::unordered_map<std::string, ScalarSpan> observation_index_;
  std::unordered_map<std::string, ScalarSpan> command_index_;
  std::size_t observation_scalar_count_ = 0;
  std::size_t command_scalar_count_ = 0;
};

struct AdapterRegistration {
  std::string adapter_id;
  std::uint64_t session_id = 0;
  std::uint64_t epoch = 0;
  ControllerLayout layout;
};

enum class FaultCode {
  kControllerError,
  kInvalidOutput,
  kInternalControllerError,
};

}  // namespace mujoco_sim::adapter_sdk
