#include "simulation_core_impl.hpp"

#include <utility>

namespace mujoco_sim::core {
namespace {

using mujoco_sim::core::ACTIVE;
using mujoco_sim::core::ERROR;
using mujoco_sim::core::OBSERVER;
using mujoco_sim::core::OK;
using mujoco_sim::core::PROVISIONAL;
using mujoco_sim::core::TIMED_OUT;
using mujoco_sim::core::UNREGISTERED;

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
  result.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return result;
}

EngineResult<mujoco_sim::core::RegisterAdapterResponse>
SimulationCore::Impl::RegisterAdapterWithEffects(
    const mujoco_sim::core::RegisterAdapterRequest& request) {
  EngineResult<mujoco_sim::core::RegisterAdapterResponse> result;
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
                : mujoco_sim::core::CONTROLLER_STATE_UNSPECIFIED,
       result.response.state(), result.response.message(), current_tick_id_});
  result.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return result;
}

EngineResult<mujoco_sim::core::ActivateAdapterResponse>
SimulationCore::Impl::ActivateAdapterWithEffects(
    const mujoco_sim::core::ActivateAdapterRequest& request) {
  EngineResult<mujoco_sim::core::ActivateAdapterResponse> result;
  result.response = DoActivateAdapter(request);
  if (result.response.code() != OK) return result;
  result.effects.adapter_changes.push_back(
      {AdapterChangeKind::kActivated, request.adapter_id(), request.role(),
       request.session_id(), std::nullopt, PROVISIONAL,
       result.response.state(), result.response.message(), current_tick_id_});
  result.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return result;
}

EngineResult<mujoco_sim::core::UnregisterAdapterResponse>
SimulationCore::Impl::UnregisterAdapterWithEffects(
    const mujoco_sim::core::UnregisterAdapterRequest& request) {
  EngineResult<mujoco_sim::core::UnregisterAdapterResponse> result;
  const auto previous = FindAdapterInfo(request.adapter_id(), request.role());
  result = DoUnregisterAdapter(request);
  if (result.response.code() != OK) return result;
  result.effects.adapter_changes.push_back(
      {AdapterChangeKind::kUnregistered, request.adapter_id(), request.role(),
       request.session_id(), std::nullopt,
       previous ? previous->state
                : mujoco_sim::core::CONTROLLER_STATE_UNSPECIFIED,
       UNREGISTERED, result.response.message(), current_tick_id_});
  result.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
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
  result.effects.status_urgency = !recovering.empty()
                                      ? EngineEffects::StatusUrgency::kLifecycle
                                      : ((!was_open && tick_open_)
                                             ? EngineEffects::StatusUrgency::kPeriodicOnly
                                             : EngineEffects::StatusUrgency::kNone);
  return result;
}

EngineEffects SimulationCore::Impl::CompleteEmptyBarrierWithEffects() {
  EngineEffects effects;
  effects.step_completed = DoCompleteEmptyBarrier();
  effects.status_urgency = effects.step_completed
                               ? EngineEffects::StatusUrgency::kPeriodicOnly
                               : EngineEffects::StatusUrgency::kNone;
  return effects;
}

EngineResult<CommitReceipt> SimulationCore::Impl::SubmitCommitWithEffects(
    const mujoco_sim::core::AdapterCommit& commit) {
  return DoSubmitCommit(commit);
}

EngineResult<CommitReceipt> SimulationCore::Impl::SubmitFaultWithEffects(
    const ControllerFault& fault) {
  return DoSubmitFault(fault);
}

EngineEffects SimulationCore::Impl::AcceptHeartbeatWithEffects(
    const mujoco_sim::core::AdapterHeartbeat& heartbeat,
    std::chrono::steady_clock::time_point now) {
  EngineEffects effects;
  DoAcceptHeartbeat(heartbeat, now);
  effects.status_urgency = EngineEffects::StatusUrgency::kPeriodicOnly;
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
  effects.status_urgency = EngineEffects::StatusUrgency::kTerminal;
  return effects;
}

EngineEffects SimulationCore::Impl::MarkStoppedWithEffects(
    const std::string& message) {
  MarkStopped(message);
  return EngineEffects{{}, std::nullopt,
                       EngineEffects::StatusUrgency::kTerminal};
}


}  // namespace mujoco_sim::core
