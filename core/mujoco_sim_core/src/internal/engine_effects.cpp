#include "simulation_core_impl.hpp"

#include <utility>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::v2::ACTIVE;
using mujoco_sim::v2::ERROR;
using mujoco_sim::v2::OBSERVER;
using mujoco_sim::v2::OK;
using mujoco_sim::v2::PROVISIONAL;
using mujoco_sim::v2::TIMED_OUT;
using mujoco_sim::v2::UNREGISTERED;

}  // namespace

EngineResult<ModelLoadReceipt> SimulationCore::Impl::LoadModelWithEffects(
    const std::string& model_path, const ModelLoadOptions& options) {
  EngineResult<ModelLoadReceipt> result;
  const auto previous = AdapterInfos();
  std::string error;
  result.response.loaded = DoLoadModel(model_path, options, &error);
  result.response.message = std::move(error);
  if (!result.response.loaded) return result;
  for (const auto& adapter : previous) {
    result.effects.adapter_changes.push_back(
        {AdapterChangeKind::kUnregistered, adapter.adapter_id, adapter.role,
         adapter.session_id, std::nullopt, adapter.state, UNREGISTERED,
         "model reloaded", current_tick_id_});
  }
  result.effects.status_changed = true;
  return result;
}

EngineResult<mujoco_sim::v2::RegisterAdapterResponse>
SimulationCore::Impl::RegisterAdapterWithEffects(
    const mujoco_sim::v2::RegisterAdapterRequest& request) {
  EngineResult<mujoco_sim::v2::RegisterAdapterResponse> result;
  const auto previous = FindAdapterInfo(request.adapter_id());
  result.response = DoRegisterAdapter(request);
  if (result.response.code() != OK) return result;
  result.effects.adapter_changes.push_back(
      {previous ? AdapterChangeKind::kReplaced
                : AdapterChangeKind::kRegistered,
       result.response.adapter_id(), result.response.role(),
       result.response.session_id(),
       previous ? std::optional(previous->session_id) : std::nullopt,
       previous ? previous->state
                : mujoco_sim::v2::CONTROLLER_STATE_UNSPECIFIED,
       result.response.state(), result.response.message(), current_tick_id_});
  result.effects.status_changed = true;
  return result;
}

EngineResult<mujoco_sim::v2::ActivateAdapterResponse>
SimulationCore::Impl::ActivateAdapterWithEffects(
    const mujoco_sim::v2::ActivateAdapterRequest& request) {
  EngineResult<mujoco_sim::v2::ActivateAdapterResponse> result;
  result.response = DoActivateAdapter(request);
  if (result.response.code() != OK) return result;
  result.effects.adapter_changes.push_back(
      {AdapterChangeKind::kActivated, request.adapter_id(), request.role(),
       request.session_id(), std::nullopt, PROVISIONAL,
       result.response.state(), result.response.message(), current_tick_id_});
  result.effects.status_changed = true;
  return result;
}

EngineResult<mujoco_sim::v2::UnregisterAdapterResponse>
SimulationCore::Impl::UnregisterAdapterWithEffects(
    const mujoco_sim::v2::UnregisterAdapterRequest& request) {
  EngineResult<mujoco_sim::v2::UnregisterAdapterResponse> result;
  const auto previous = FindAdapterInfo(request.adapter_id(), request.role());
  result = DoUnregisterAdapter(request);
  if (result.response.code() != OK) return result;
  result.effects.adapter_changes.push_back(
      {AdapterChangeKind::kUnregistered, request.adapter_id(), request.role(),
       request.session_id(), std::nullopt,
       previous ? previous->state
                : mujoco_sim::v2::CONTROLLER_STATE_UNSPECIFIED,
       UNREGISTERED, result.response.message(), current_tick_id_});
  result.effects.status_changed = true;
  return result;
}

EngineResult<std::vector<IssuedTickHandle>>
SimulationCore::Impl::OpenIssuedTicksWithEffects() {
  EngineResult<std::vector<IssuedTickHandle>> result;
  const bool was_open = tick_open_;
  const auto recovering = RecoveryPendingControllers();
  result.response = DoOpenIssuedTicks();
  for (const auto& adapter : recovering) {
    result.effects.adapter_changes.push_back(
        {AdapterChangeKind::kRecovered, adapter.adapter_id, adapter.role,
         adapter.session_id, std::nullopt, TIMED_OUT, ACTIVE, {},
         current_tick_id_});
  }
  result.effects.status_changed = (!was_open && tick_open_) ||
                                  !recovering.empty();
  return result;
}

EngineEffects SimulationCore::Impl::CompleteEmptyBarrierWithEffects() {
  EngineEffects effects;
  effects.step_completed = DoCompleteEmptyBarrier();
  effects.status_changed = effects.step_completed.has_value();
  return effects;
}

EngineResult<CommitReceipt> SimulationCore::Impl::SubmitCommitWithEffects(
    const mujoco_sim::v2::AdapterCommit& commit) {
  return DoSubmitCommit(commit);
}

EngineEffects SimulationCore::Impl::AcceptHeartbeatWithEffects(
    const mujoco_sim::v2::AdapterHeartbeat& heartbeat,
    std::chrono::steady_clock::time_point now) {
  EngineEffects effects;
  const auto previous_tick = ControllerHeartbeatTick(heartbeat.adapter_id());
  const auto previous_time = ObserverHeartbeatTime(heartbeat.adapter_id());
  DoAcceptHeartbeat(heartbeat, now);
  if (heartbeat.role() == mujoco_sim::v2::CONTROLLER) {
    effects.status_changed =
        FindAdapterInfo(heartbeat.adapter_id(), mujoco_sim::v2::CONTROLLER)
            .has_value() &&
        ControllerHeartbeatTick(heartbeat.adapter_id()) != previous_tick;
  } else if (heartbeat.role() == OBSERVER) {
    effects.status_changed =
        FindAdapterInfo(heartbeat.adapter_id(), OBSERVER).has_value() &&
        ObserverHeartbeatTime(heartbeat.adapter_id()) != previous_time;
  }
  return effects;
}

EngineEffects SimulationCore::Impl::CheckDeadlinesWithEffects(
    std::chrono::steady_clock::time_point now) {
  return DoCheckDeadlines(now);
}

EngineEffects SimulationCore::Impl::MarkFailedWithEffects(
    const std::string& message) {
  const auto previous = AdapterInfos();
  EnterFatalError(message);
  EngineEffects effects;
  for (const auto& adapter : previous) {
    const auto current = FindAdapterInfo(adapter.adapter_id, adapter.role);
    if (current && current->state != adapter.state) {
      effects.adapter_changes.push_back(
          {AdapterChangeKind::kServerFailed, adapter.adapter_id, adapter.role,
           adapter.session_id, std::nullopt, adapter.state, current->state,
           message, current_tick_id_});
    }
  }
  effects.status_changed = true;
  return effects;
}

EngineEffects SimulationCore::Impl::MarkStoppedWithEffects(
    const std::string& message) {
  MarkStopped(message);
  return EngineEffects{{}, std::nullopt, true};
}


}  // namespace mujoco_sim::core
