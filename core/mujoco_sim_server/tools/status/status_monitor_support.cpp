#include "status_monitor_support.hpp"

#include <utility>
#include <vector>

#include <ecal/service/client.h>
#include <mujoco_sim/protocol/topics.hpp>

namespace mujoco_sim::server::status_tool::internal {
namespace {

constexpr int kResourceCallTimeoutMs = 100;

bool SameProcess(const PublisherIdentity& publisher, const eCAL::SServiceId& service) {
  return !publisher.host_name.empty() && publisher.process_id != 0 &&
         publisher.process_id == service.service_id.process_id &&
         publisher.host_name == service.service_id.host_name;
}

}  // namespace

ResourceQueryResult QueryResources(eCAL::CServiceClient* client, const PublisherIdentity& publisher,
                                   std::uint64_t epoch) {
  if (client == nullptr || !client->IsConnected()) {
    return {std::nullopt, "registry service is unavailable"};
  }
  mujoco_sim::v4::GetAvailableResourcesRequest request;
  std::string bytes;
  if (!request.SerializeToString(&bytes)) {
    return {std::nullopt, "failed to serialize resource request"};
  }

  eCAL::ServiceResponseVecT responses;
  if (!client->CallWithResponse(mujoco_sim::protocol::kGetAvailableResourcesMethod, bytes,
                                responses, kResourceCallTimeoutMs)) {
    return {std::nullopt, "registry service call failed"};
  }
  std::vector<mujoco_sim::v4::GetAvailableResourcesResponse> valid;
  std::string matching_error;
  for (const auto& response : responses) {
    if (!SameProcess(publisher, response.server_id)) continue;
    if (response.call_state != eCAL::eCallState::executed) {
      matching_error =
          response.error_msg.empty() ? "resource service call did not execute" : response.error_msg;
      continue;
    }
    if (response.ret_state != 0) {
      matching_error =
          "resource service returned transport error " + std::to_string(response.ret_state);
      continue;
    }
    mujoco_sim::v4::GetAvailableResourcesResponse parsed;
    if (!parsed.ParseFromString(response.response)) {
      matching_error = "failed to parse resource response";
      continue;
    }
    if (parsed.code() != mujoco_sim::v4::OK) {
      matching_error = parsed.message().empty() ? mujoco_sim::v4::ErrorCode_Name(parsed.code())
                                                : parsed.message();
      continue;
    }
    if (parsed.epoch() != epoch) {
      matching_error = "resource response epoch does not match status";
      continue;
    }
    valid.push_back(std::move(parsed));
  }
  if (valid.size() == 1) return {std::move(valid.front()), {}};
  if (valid.size() > 1) {
    return {std::nullopt, "multiple registry services match the status publisher"};
  }
  return {std::nullopt, matching_error.empty()
                            ? "registry response did not match the status publisher"
                            : std::move(matching_error)};
}

bool StatusCallbackGate::Enter() {
  std::lock_guard lock(mutex_);
  if (!accepting_) return false;
  ++active_;
  return true;
}

void StatusCallbackGate::Leave() noexcept {
  try {
    {
      std::lock_guard lock(mutex_);
      if (active_ > 0) --active_;
    }
    condition_.notify_all();
  } catch (...) {
  }
}

void StatusCallbackGate::StopAccepting() {
  std::lock_guard lock(mutex_);
  accepting_ = false;
}

void StatusCallbackGate::WaitForIdle() {
  std::unique_lock lock(mutex_);
  condition_.wait(lock, [this] { return active_ == 0; });
}

StatusCallbackLease::StatusCallbackLease(std::shared_ptr<StatusCallbackGate> gate)
    : gate_(std::move(gate)), entered_(gate_ && gate_->Enter()) {}

StatusCallbackLease::~StatusCallbackLease() noexcept {
  if (entered_) gate_->Leave();
}

}  // namespace mujoco_sim::server::status_tool::internal
