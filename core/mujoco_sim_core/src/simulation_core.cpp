#include <mujoco_sim/core/simulation_core.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <mujoco/mujoco.h>
#include "internal/model_runtime.hpp"
#include "internal/layout_fingerprint.hpp"
#include "internal/resource_catalog.hpp"
#include "internal/snapshot_manager.hpp"
namespace mujoco_sim::core {
namespace {
bool DeliveryValid(const ObservationDelivery& delivery) {
  return (delivery.has_every_n_steps() && delivery.every_n_steps() > 0) ||
         (delivery.has_period_s() && std::isfinite(delivery.period_s()) &&
          delivery.period_s() > 0.0);
}
bool AdapterIdValid(std::string_view value) {
  if (value.empty() || value.size() > 64) return false;
  const auto alnum = [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
  };
  if (!alnum(value.front())) return false;
  return std::all_of(value.begin() + 1, value.end(),
                     [&](char c) { return alnum(c) || c == '_' || c == '-'; });
}
bool FaultCodeValid(FaultCode code) {
  return code == FaultCode::kControllerError || code == FaultCode::kInvalidOutput ||
         code == FaultCode::kInternalControllerError;
}
AdapterChange Change(AdapterChangeKind kind, std::string id, AdapterRole role,
                     std::uint64_t session, AdapterState old_state, AdapterState new_state,
                     std::uint64_t tick, std::string message = {}) {
  return {kind,      std::move(id),      role, session, std::nullopt, old_state,
          new_state, std::move(message), tick};
}
}  // namespace
class SimulationCore::Impl {
 public:
  struct Controller {
    std::string id;
    std::uint64_t session = 0;
    AdapterState state = PROVISIONAL;
    ResourceLayout layout;
    ControllerRuntimePolicy policy;
    std::chrono::steady_clock::time_point registered_at{}, activated_at{}, last_command_at{},
        offline_at{};
    std::uint64_t activation_tick = 0, state_since_tick = 0, state_sequence = 0,
                  last_command_sequence = 0, timeout_count = 0, next_period = 1;
    std::vector<double> last_command;
    bool command_dirty = false;
    std::optional<ControlPolicy> pending_policy;
    std::string message;
  };
  struct Observer {
    std::string id;
    std::uint64_t session = 0;
    AdapterState state = PROVISIONAL;
    ResourceLayout layout;
    ObservationDelivery delivery;
    std::uint32_t timeout_ms = 1000;
    std::chrono::steady_clock::time_point registered_at{}, last_heartbeat_at{};
    std::uint64_t activation_tick = 0, state_since_tick = 0, sequence = 0, timeout_count = 0,
                  next_period = 1;
    std::optional<std::uint64_t> last_delivery_tick;
    std::string message;
  };
  explicit Impl(CoreOptions value)
      : options(std::move(value)),
        model(std::make_unique<internal::ModelRuntime>()),
        catalog(std::make_unique<internal::ResourceCatalog>()),
        snapshots(std::make_unique<detail::SnapshotManager>()) {}
  ResourceLayout MakeLayout(const RegisterAdapterRequest& request, ErrorCode* code,
                            std::string* message) const {
    ResourceLayout result;
    std::unordered_set<std::string> names;
    for (const auto& name : request.observation_names()) {
      if (!names.insert(name).second) {
        *code = INVALID_ARGUMENT;
        *message = "duplicate observation: " + name;
        return {};
      }
      const auto* descriptor = catalog->FindObservation(name);
      if (!descriptor) {
        *code = NOT_FOUND;
        *message = "unknown observation sensor: " + name;
        return {};
      }
      result.observation_spans.push_back(
          {result.observation_scalar_count, descriptor->dimension()});
      result.observation_scalar_count += descriptor->dimension();
      result.observations_.push_back(*descriptor);
    }
    names.clear();
    for (const auto& name : request.command_names()) {
      if (!names.insert(name).second) {
        *code = INVALID_ARGUMENT;
        *message = "duplicate command: " + name;
        return {};
      }
      const auto* descriptor = catalog->FindCommand(name);
      if (!descriptor) {
        *code = NOT_FOUND;
        *message = "unknown command actuator: " + name;
        return {};
      }
      result.command_spans.push_back({result.command_scalar_count, descriptor->dimension()});
      result.command_scalar_count += descriptor->dimension();
      result.commands_.push_back(*descriptor);
    }
    result.fingerprint = internal::ComputeLayoutFingerprint(result);
    *code = OK;
    return result;
  }
  std::vector<double> Read(const ResourceLayout& layout) const {
    std::vector<double> values;
    values.reserve(layout.observation_scalar_count);
    const mjModel* m = model->model_ptr();
    const mjData* d = model->data_ptr();
    for (const auto& observation : layout.observations()) {
      const int sensor = observation.mujoco_id();
      const int address = m->sensor_adr[sensor];
      for (int i = 0; i < m->sensor_dim[sensor]; ++i)
        values.push_back(static_cast<double>(d->sensordata[address + i]));
    }
    return values;
  }
  bool Due(const ObservationDelivery& delivery, std::uint64_t activation_tick,
           std::uint64_t next_period) const {
    if (delivery.has_every_n_steps())
      return current_tick > activation_tick &&
             (current_tick - activation_tick) % delivery.every_n_steps() == 0;
    const long double elapsed =
        static_cast<long double>(current_tick - activation_tick) * model->timestep();
    return elapsed + 1e-15L >= static_cast<long double>(next_period) * delivery.period_s();
  }
  ObservationDelivery EffectiveDelivery(ObservationDelivery delivery) const {
    const double minimum_period = 1.0 / options.controller_state_max_rate_hz;
    if (delivery.has_period_s()) {
      delivery.set_period_s(std::max(delivery.period_s(), minimum_period));
      return delivery;
    }
    const auto minimum_steps =
        static_cast<std::uint32_t>(std::max(1.0, std::ceil(minimum_period / model->timestep())));
    delivery.set_every_n_steps(std::max(delivery.every_n_steps(), minimum_steps));
    return delivery;
  }
  ControllerRuntimePolicy PolicyFor(std::string_view adapter_id) const {
    ControllerRuntimePolicy policy;
    policy.command_timeout_ms = options.command_timeout_ms;
    policy.command_keepalive_ms = options.command_keepalive_ms;
    policy.offline_session_expire_ms = options.offline_session_expire_ms;
    policy.state_delivery = EffectiveDelivery(options.controller_state_delivery);
    policy.startup_policy = options.startup_policy;
    policy.offline_policy = options.offline_policy;
    policy.error_policy = options.error_policy;
    policy.release_policy = options.release_policy;
    const auto found =
        std::find_if(options.controller_overrides.begin(), options.controller_overrides.end(),
                     [adapter_id](const auto& value) { return value.adapter_id == adapter_id; });
    if (found == options.controller_overrides.end()) return policy;
    policy.command_timeout_ms = found->command_timeout_ms;
    policy.command_keepalive_ms = found->command_keepalive_ms;
    policy.offline_session_expire_ms = found->offline_session_expire_ms;
    policy.state_delivery = EffectiveDelivery(found->state_delivery);
    policy.startup_policy = found->startup_policy;
    policy.offline_policy = found->offline_policy;
    policy.error_policy = found->error_policy;
    policy.release_policy = found->release_policy;
    return policy;
  }
  void QueuePolicy(const Controller& controller, ControlPolicy policy) {
    if (policy != ControlPolicy::kZero) return;
    for (const auto& command : controller.layout.commands())
      queued_policy_writes.push_back({command.mujoco_id(), 0.0});
  }
  bool CommandValuesValid(const Controller& controller, const ControllerCommand& command) const {
    for (std::size_t resource = 0; resource < controller.layout.commands_.size(); ++resource) {
      const auto& descriptor = controller.layout.commands_[resource];
      const auto span = controller.layout.command_spans[resource];
      for (std::size_t index = span.offset; index < span.offset + span.dimension; ++index) {
        const double value = command.commands[index];
        if (!std::isfinite(value)) return false;
        if (descriptor.has_ctrl_range() &&
            (value < descriptor.ctrl_min() || value > descriptor.ctrl_max()))
          return false;
      }
    }
    return true;
  }
  void ApplyControllerWrites() {
    if (!queued_policy_writes.empty()) {
      model->ApplyActuatorWrites(queued_policy_writes);
      queued_policy_writes.clear();
    }
    for (auto& [id, controller] : controllers) {
      if (controller.command_dirty) {
        std::vector<internal::ValidatedActuatorWrite> writes;
        for (std::size_t index = 0; index < controller.last_command.size(); ++index)
          writes.push_back(
              {controller.layout.commands_[index].mujoco_id(), controller.last_command[index]});
        model->ApplyActuatorWrites(writes);
        controller.command_dirty = false;
        controller.pending_policy.reset();
      } else if (controller.pending_policy) {
        if (*controller.pending_policy == ControlPolicy::kZero) {
          std::vector<int> ids;
          for (const auto& descriptor : controller.layout.commands_)
            ids.push_back(descriptor.mujoco_id());
          model->ZeroActuators(ids);
        }
        controller.pending_policy.reset();
      }
    }
  }
  std::vector<ControllerStateFrame> BuildControllerStates(double time) {
    std::vector<ControllerStateFrame> frames;
    for (auto& [id, controller] : controllers) {
      const bool publishes = controller.state == ACTIVE_NO_COMMAND || controller.state == ONLINE ||
                             controller.state == OFFLINE;
      if (!publishes || !Due(controller.policy.state_delivery, controller.activation_tick,
                             controller.next_period))
        continue;
      ControllerStateFrame frame;
      frame.identity = {
          {controller.session, epoch}, current_tick, controller.layout.fingerprint, time};
      frame.sequence = ++controller.state_sequence;
      frame.observations = Read(controller.layout);
      frames.push_back(std::move(frame));
      if (controller.policy.state_delivery.has_period_s()) ++controller.next_period;
    }
    return frames;
  }
  std::vector<ObservationFrame> BuildObserverFrames(double time) {
    std::vector<ObservationFrame> frames;
    for (auto& [id, observer] : observers) {
      if (observer.state != ONLINE) continue;
      const bool first_delivery = !observer.last_delivery_tick;
      const bool due =
          first_delivery ||
          (observer.delivery.has_every_n_steps()
               ? current_tick - *observer.last_delivery_tick >= observer.delivery.every_n_steps()
               : Due(observer.delivery, observer.activation_tick, observer.next_period));
      if (!due) continue;
      ObservationFrame frame;
      frame.identity = {{observer.session, epoch}, current_tick, observer.layout.fingerprint, time};
      frame.sequence = ++observer.sequence;
      frame.observations = Read(observer.layout);
      frames.push_back(std::move(frame));
      observer.last_delivery_tick = current_tick;
      if (observer.delivery.has_period_s()) {
        if (first_delivery)
          observer.activation_tick = current_tick;
        else
          ++observer.next_period;
      }
    }
    return frames;
  }
  void CheckControllerDeadlines(std::chrono::steady_clock::time_point now, EngineEffects* effects,
                                std::vector<std::string>* erase) {
    for (auto& [id, controller] : controllers) {
      if (controller.state == PROVISIONAL &&
          now - controller.registered_at >
              std::chrono::milliseconds(options.activation_timeout_ms)) {
        erase->push_back(id);
        effects->adapter_changes.push_back(Change(AdapterChangeKind::kActivationExpired, id,
                                                  CONTROLLER, controller.session, PROVISIONAL,
                                                  UNREGISTERED, current_tick));
        continue;
      }
      const bool command_live = controller.state == ACTIVE_NO_COMMAND || controller.state == ONLINE;
      if (command_live && now - controller.last_command_at >
                              std::chrono::milliseconds(controller.policy.command_timeout_ms)) {
        const auto old = controller.state;
        controller.state = OFFLINE;
        controller.offline_at = now;
        controller.state_since_tick = current_tick;
        ++controller.timeout_count;
        controller.message = "controller command timed out";
        controller.pending_policy = controller.policy.offline_policy;
        controller.command_dirty = false;
        effects->adapter_changes.push_back(Change(AdapterChangeKind::kTimedOut, id, CONTROLLER,
                                                  controller.session, old, OFFLINE, current_tick,
                                                  controller.message));
      } else if (controller.state == OFFLINE && controller.policy.offline_session_expire_ms > 0 &&
                 now - controller.offline_at >
                     std::chrono::milliseconds(controller.policy.offline_session_expire_ms)) {
        erase->push_back(id);
        QueuePolicy(controller, controller.policy.release_policy);
        effects->adapter_changes.push_back(Change(AdapterChangeKind::kUnregistered, id, CONTROLLER,
                                                  controller.session, OFFLINE, UNREGISTERED,
                                                  current_tick, "offline session expired"));
      }
    }
  }
  void CheckObserverDeadlines(std::chrono::steady_clock::time_point now, EngineEffects* effects,
                              std::vector<std::string>* erase) {
    for (auto& [id, observer] : observers) {
      if (observer.state == PROVISIONAL &&
          now - observer.registered_at > std::chrono::milliseconds(options.activation_timeout_ms)) {
        erase->push_back(id);
        effects->adapter_changes.push_back(Change(AdapterChangeKind::kActivationExpired, id,
                                                  OBSERVER, observer.session, PROVISIONAL,
                                                  UNREGISTERED, current_tick));
      } else if (observer.state == ONLINE && now - observer.last_heartbeat_at >
                                                 std::chrono::milliseconds(observer.timeout_ms)) {
        observer.state = OFFLINE;
        observer.state_since_tick = current_tick;
        ++observer.timeout_count;
        observer.message = "observer heartbeat timed out";
        effects->adapter_changes.push_back(Change(AdapterChangeKind::kTimedOut, id, OBSERVER,
                                                  observer.session, ONLINE, OFFLINE, current_tick,
                                                  observer.message));
      }
    }
  }
  EngineResult<RegisterAdapterResponse> RegisterController(
      const RegisterAdapterRequest& request, ResourceLayout layout,
      std::chrono::steady_clock::time_point now) {
    EngineResult<RegisterAdapterResponse> out;
    auto& response = out.response;
    if (layout.commands_.empty()) {
      response.set_code(INVALID_ARGUMENT);
      response.set_message("controller requires at least one command");
      return out;
    }
    auto existing = controllers.find(request.adapter_id());
    if (observers.contains(request.adapter_id()) ||
        (existing != controllers.end() && existing->second.state != OFFLINE &&
         existing->second.state != ERROR)) {
      response.set_code(ALREADY_EXISTS);
      response.set_message("adapter ID is already registered");
      return out;
    }
    for (const auto& descriptor : layout.commands_) {
      const auto owner = owners.find(descriptor.name());
      if (owner != owners.end() && owner->second != request.adapter_id()) {
        response.set_code(RESOURCE_CONFLICT);
        response.set_message("command resource is already owned: " + descriptor.name());
        return out;
      }
    }
    std::optional<std::uint64_t> replaced;
    if (existing != controllers.end()) {
      replaced = existing->second.session;
      const auto old_state = existing->second.state;
      QueuePolicy(existing->second, existing->second.policy.release_policy);
      sessions.erase(existing->second.session);
      for (const auto& descriptor : existing->second.layout.commands_)
        owners.erase(descriptor.name());
      controllers.erase(existing);
      out.effects.adapter_changes.push_back(
          Change(AdapterChangeKind::kUnregistered, request.adapter_id(), CONTROLLER, *replaced,
                 old_state, UNREGISTERED, current_tick, "session replaced"));
    }
    Controller controller;
    controller.id = request.adapter_id();
    controller.session = next_session++;
    controller.layout = std::move(layout);
    controller.registered_at = now;
    controller.state_since_tick = current_tick;
    controller.policy = PolicyFor(request.adapter_id());
    const auto session = controller.session;
    for (const auto& descriptor : controller.layout.commands_)
      owners[descriptor.name()] = controller.id;
    sessions[session] = {CONTROLLER, controller.id};
    controllers.emplace(controller.id, std::move(controller));
    auto& stored = controllers.at(request.adapter_id());
    response.set_adapter_id(stored.id);
    response.set_role(CONTROLLER);
    response.set_session_id(session);
    response.set_epoch(epoch);
    response.layout_ = stored.layout;
    response.set_state(PROVISIONAL);
    response.controller_policy = stored.policy;
    auto change = Change(replaced ? AdapterChangeKind::kReplaced : AdapterChangeKind::kRegistered,
                         stored.id, CONTROLLER, session, UNREGISTERED, PROVISIONAL, current_tick);
    change.previous_session_id = replaced;
    out.effects.adapter_changes.push_back(std::move(change));
    return out;
  }
  EngineResult<RegisterAdapterResponse> RegisterObserver(
      const RegisterAdapterRequest& request, ResourceLayout layout,
      std::chrono::steady_clock::time_point now) {
    EngineResult<RegisterAdapterResponse> out;
    auto& response = out.response;
    if (!layout.commands_.empty() || layout.observations_.empty() ||
        !DeliveryValid(request.delivery())) {
      response.set_code(INVALID_ARGUMENT);
      response.set_message("invalid observer resources or delivery");
      return out;
    }
    if (controllers.contains(request.adapter_id()) || observers.contains(request.adapter_id())) {
      response.set_code(ALREADY_EXISTS);
      response.set_message("adapter ID is already registered");
      return out;
    }
    Observer observer;
    observer.id = request.adapter_id();
    observer.session = next_session++;
    observer.layout = std::move(layout);
    observer.delivery = request.delivery();
    observer.timeout_ms = request.session_timeout_ms() ? request.session_timeout_ms()
                                                       : options.observer_session_timeout_ms;
    observer.registered_at = now;
    observer.last_heartbeat_at = now;
    observer.state_since_tick = current_tick;
    const auto session = observer.session;
    sessions[session] = {OBSERVER, observer.id};
    observers.emplace(observer.id, std::move(observer));
    auto& stored = observers.at(request.adapter_id());
    response.set_adapter_id(stored.id);
    response.set_role(OBSERVER);
    response.set_session_id(session);
    response.set_epoch(epoch);
    response.layout_ = stored.layout;
    response.set_state(PROVISIONAL);
    out.effects.adapter_changes.push_back(Change(AdapterChangeKind::kRegistered, stored.id,
                                                 OBSERVER, session, UNREGISTERED, PROVISIONAL,
                                                 current_tick));
    return out;
  }
  CoreOptions options;
  std::unique_ptr<internal::ModelRuntime> model;
  std::unique_ptr<internal::ResourceCatalog> catalog;
  std::unique_ptr<detail::SnapshotManager> snapshots;
  std::unordered_map<std::string, Controller> controllers;
  std::unordered_map<std::string, Observer> observers;
  std::unordered_map<std::uint64_t, std::pair<AdapterRole, std::string>> sessions;
  std::unordered_map<std::string, std::string> owners;
  std::vector<internal::ValidatedActuatorWrite> queued_policy_writes;
  std::uint64_t next_session = 1, epoch = 1, current_tick = 0;
  ServerState server_state = IDLE;
  std::string status_message;
};
SimulationCore::SimulationCore(CoreOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}
SimulationCore::~SimulationCore() = default;
SimulationCore::SimulationCore(SimulationCore&&) noexcept = default;
SimulationCore& SimulationCore::operator=(SimulationCore&&) noexcept = default;
EngineResult<ModelLoadReceipt> SimulationCore::LoadModel(const std::string& path,
                                                         const ModelLoadOptions& options) {
  EngineResult<ModelLoadReceipt> out;
  auto prepared = internal::ModelRuntime::Prepare(path, options, &out.response.message);
  if (!prepared) return out;
  auto catalog = internal::ResourceCatalog::Build(*prepared->model);
  impl_->model->Commit(std::move(*prepared));
  *impl_->catalog = std::move(catalog);
  impl_->snapshots->Reset();
  impl_->controllers.clear();
  impl_->observers.clear();
  impl_->sessions.clear();
  impl_->owners.clear();
  impl_->queued_policy_writes.clear();
  ++impl_->epoch;
  impl_->current_tick = 0;
  impl_->server_state = RUNNING;
  impl_->status_message.clear();
  out.response.loaded = true;
  out.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return out;
}
bool SimulationCore::IsModelLoaded() const {
  return impl_->model->loaded();
}
std::unique_ptr<SimulationSnapshot> SimulationCore::CreateSnapshot(std::string* e) const {
  return impl_->snapshots->Create(*impl_->model, e);
}
std::unique_ptr<SimulationSnapshot> SimulationCore::CloneSnapshot(const SimulationSnapshot& s,
                                                                  std::string* e) const {
  return impl_->snapshots->Clone(*impl_->model, s, e);
}
bool SimulationCore::UpdateSnapshot(SimulationSnapshot& s, std::string* e) const {
  return impl_->snapshots->Update(*impl_->model, &s, e);
}
CoreResources SimulationCore::resources() const {
  return {impl_->catalog->observations(), impl_->catalog->commands()};
}
GetAvailableResourcesResponse SimulationCore::GetAvailableResources(
    const GetAvailableResourcesRequest&) const {
  GetAvailableResourcesResponse r;
  if (!IsModelLoaded()) {
    r.set_code(FAILED_PRECONDITION);
    r.set_message("model is not loaded");
    return r;
  }
  r.set_epoch(impl_->epoch);
  for (const auto& d : impl_->catalog->observations())
    *r.add_observations() = d;
  for (const auto& d : impl_->catalog->commands())
    *r.add_commands() = d;
  return r;
}
EngineResult<RegisterAdapterResponse> SimulationCore::RegisterAdapter(
    const RegisterAdapterRequest& request, std::chrono::steady_clock::time_point now) {
  EngineResult<RegisterAdapterResponse> out;
  auto& r = out.response;
  if (!IsModelLoaded()) {
    r.set_code(FAILED_PRECONDITION);
    r.set_message("model is not loaded");
    return out;
  }
  if (!AdapterIdValid(request.adapter_id()) ||
      (request.role() != CONTROLLER && request.role() != OBSERVER)) {
    r.set_code(INVALID_ARGUMENT);
    r.set_message("invalid adapter registration");
    return out;
  }
  ErrorCode code;
  std::string message;
  auto layout = impl_->MakeLayout(request, &code, &message);
  if (code != OK) {
    r.set_code(code);
    r.set_message(std::move(message));
    return out;
  }
  out = request.role() == CONTROLLER ? impl_->RegisterController(request, std::move(layout), now)
                                     : impl_->RegisterObserver(request, std::move(layout), now);
  out.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return out;
}
EngineResult<ActivateAdapterResponse> SimulationCore::ActivateAdapter(
    const ActivateAdapterRequest& request, std::chrono::steady_clock::time_point now) {
  EngineResult<ActivateAdapterResponse> out;
  auto& r = out.response;
  if (request.epoch() != impl_->epoch) {
    r.set_code(INVALID_EPOCH);
    r.set_message("epoch mismatch");
    return out;
  }
  auto it = impl_->sessions.find(request.session_id());
  if (it == impl_->sessions.end()) {
    r.set_code(INVALID_SESSION);
    r.set_message("unknown session");
    return out;
  }
  if (it->second.first == CONTROLLER) {
    auto& c = impl_->controllers.at(it->second.second);
    if (c.state != PROVISIONAL) {
      r.set_code(FAILED_PRECONDITION);
      r.set_message("controller is not provisional");
      return out;
    }
    c.state = ACTIVE_NO_COMMAND;
    c.state_since_tick = impl_->current_tick;
    c.activated_at = now;
    c.last_command_at = now;
    c.activation_tick = impl_->current_tick;
    c.pending_policy = c.policy.startup_policy;
    r.set_state(c.state);
    out.effects.adapter_changes.push_back(Change(AdapterChangeKind::kActivated, c.id, CONTROLLER,
                                                 c.session, PROVISIONAL, c.state,
                                                 impl_->current_tick));
  } else {
    auto& o = impl_->observers.at(it->second.second);
    if (o.state != PROVISIONAL) {
      r.set_code(FAILED_PRECONDITION);
      r.set_message("observer is not provisional");
      return out;
    }
    o.state = ONLINE;
    o.state_since_tick = impl_->current_tick;
    o.activation_tick = impl_->current_tick;
    o.last_heartbeat_at = now;
    r.set_state(ONLINE);
    out.effects.adapter_changes.push_back(Change(AdapterChangeKind::kActivated, o.id, OBSERVER,
                                                 o.session, PROVISIONAL, ONLINE,
                                                 impl_->current_tick));
  }
  out.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return out;
}
EngineResult<UnregisterAdapterResponse> SimulationCore::UnregisterAdapter(
    const UnregisterAdapterRequest& request) {
  EngineResult<UnregisterAdapterResponse> out;
  auto& r = out.response;
  if (request.epoch() != impl_->epoch) {
    r.set_code(INVALID_EPOCH);
    return out;
  }
  auto it = impl_->sessions.find(request.session_id());
  if (it == impl_->sessions.end()) {
    r.set_code(INVALID_SESSION);
    return out;
  }
  const auto [role, id] = it->second;
  AdapterState old;
  if (role == CONTROLLER) {
    auto c = std::move(impl_->controllers.at(id));
    old = c.state;
    impl_->QueuePolicy(c, c.policy.release_policy);
    for (const auto& d : c.layout.commands_)
      impl_->owners.erase(d.name());
    impl_->controllers.erase(id);
  } else {
    old = impl_->observers.at(id).state;
    impl_->observers.erase(id);
  }
  impl_->sessions.erase(it);
  r.set_state(UNREGISTERED);
  out.effects.adapter_changes.push_back(Change(AdapterChangeKind::kUnregistered, id, role,
                                               request.session_id(), old, UNREGISTERED,
                                               impl_->current_tick));
  out.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return out;
}
EngineResult<CommandReceipt> SimulationCore::SubmitCommand(
    const ControllerCommand& command, std::chrono::steady_clock::time_point now) {
  EngineResult<CommandReceipt> out;
  auto reject = [&](ErrorCode code, std::string msg) {
    out.response.code = code;
    out.response.message = std::move(msg);
    return out;
  };
  if (command.session.epoch != impl_->epoch) return reject(INVALID_EPOCH, "epoch mismatch");
  auto session = impl_->sessions.find(command.session.session_id);
  if (session == impl_->sessions.end() || session->second.first != CONTROLLER)
    return reject(INVALID_SESSION, "unknown controller session");
  auto& c = impl_->controllers.at(session->second.second);
  if (c.state == PROVISIONAL || c.state == ERROR)
    return reject(FAILED_PRECONDITION, "controller cannot accept commands in current state");
  if (command.layout != c.layout.fingerprint)
    return reject(INVALID_COMMAND, "command layout mismatch");
  if (command.sequence == 0 || command.sequence <= c.last_command_sequence)
    return reject(INVALID_COMMAND, "command sequence is not strictly increasing");
  if (command.commands.size() != c.layout.command_scalar_count)
    return reject(INVALID_COMMAND, "command scalar count mismatch");
  if (!impl_->CommandValuesValid(c, command))
    return reject(INVALID_COMMAND, "command contains invalid value");
  const AdapterState old = c.state;
  c.last_command = command.commands;
  c.last_command_sequence = command.sequence;
  c.last_command_at = now;
  c.command_dirty = true;
  c.pending_policy.reset();
  c.message.clear();
  if (c.state == ACTIVE_NO_COMMAND || c.state == OFFLINE) {
    c.state = ONLINE;
    c.state_since_tick = impl_->current_tick;
    if (old == OFFLINE)
      out.effects.adapter_changes.push_back(Change(AdapterChangeKind::kRecovered, c.id, CONTROLLER,
                                                   c.session, old, ONLINE, impl_->current_tick));
    out.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  }
  return out;
}
EngineResult<CommandReceipt> SimulationCore::SubmitFault(const ControllerFault& fault) {
  EngineResult<CommandReceipt> out;
  if (fault.session.epoch != impl_->epoch) {
    out.response.code = INVALID_EPOCH;
    return out;
  }
  auto s = impl_->sessions.find(fault.session.session_id);
  if (s == impl_->sessions.end() || s->second.first != CONTROLLER) {
    out.response.code = INVALID_SESSION;
    return out;
  }
  if (!FaultCodeValid(fault.code)) {
    out.response.code = INVALID_ARGUMENT;
    out.response.message = "invalid ControllerFault code";
    return out;
  }
  auto& c = impl_->controllers.at(s->second.second);
  if (c.state == ERROR || c.state == PROVISIONAL) {
    out.response.code = FAILED_PRECONDITION;
    return out;
  }
  auto old = c.state;
  c.state = ERROR;
  c.state_since_tick = impl_->current_tick;
  c.message = fault.message;
  c.pending_policy = c.policy.error_policy;
  c.command_dirty = false;
  out.effects.adapter_changes.push_back(Change(AdapterChangeKind::kLocalError, c.id, CONTROLLER,
                                               c.session, old, ERROR, impl_->current_tick,
                                               c.message));
  out.effects.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return out;
}
EngineEffects SimulationCore::AcceptHeartbeat(const AdapterHeartbeat& heartbeat,
                                              std::chrono::steady_clock::time_point now) {
  EngineEffects out;
  if (heartbeat.session.epoch != impl_->epoch) return out;
  auto s = impl_->sessions.find(heartbeat.session.session_id);
  if (s == impl_->sessions.end() || s->second.first != OBSERVER) return out;
  auto& o = impl_->observers.at(s->second.second);
  if (o.state == ONLINE && heartbeat.last_seen <= o.sequence) o.last_heartbeat_at = now;
  return out;
}
EngineEffects SimulationCore::Step() {
  return Step(InteractiveStepInput{});
}

EngineEffects SimulationCore::Step(const InteractiveStepInput& input) {
  EngineEffects out;
  if (!IsModelLoaded() || impl_->server_state != RUNNING) return out;
  impl_->ApplyControllerWrites();
  impl_->model->ApplyInteractiveActuatorWrites(input.actuator_writes);
  if (input.replace_body_wrenches) {
    impl_->model->ReplaceInteractiveBodyWrenches(input.body_wrenches);
  }
  impl_->model->Step();
  ++impl_->current_tick;
  StepCompletedEvent event;
  event.completed_tick_id = impl_->current_tick;
  event.next_tick_id = impl_->current_tick + 1;
  event.simulation_time_s = impl_->model->simulation_time();
  event.controller_states = impl_->BuildControllerStates(event.simulation_time_s);
  event.observer_frames = impl_->BuildObserverFrames(event.simulation_time_s);
  out.step_completed = std::move(event);
  out.status_urgency = EngineEffects::StatusUrgency::kPeriodicOnly;
  return out;
}
EngineEffects SimulationCore::CheckDeadlines(std::chrono::steady_clock::time_point now) {
  EngineEffects out;
  std::vector<std::string> erase_controllers, erase_observers;
  impl_->CheckControllerDeadlines(now, &out, &erase_controllers);
  impl_->CheckObserverDeadlines(now, &out, &erase_observers);
  for (const auto& id : erase_controllers) {
    auto& c = impl_->controllers.at(id);
    impl_->sessions.erase(c.session);
    for (const auto& d : c.layout.commands_)
      impl_->owners.erase(d.name());
    impl_->controllers.erase(id);
  }
  for (const auto& id : erase_observers) {
    impl_->sessions.erase(impl_->observers.at(id).session);
    impl_->observers.erase(id);
  }
  if (!out.adapter_changes.empty()) out.status_urgency = EngineEffects::StatusUrgency::kLifecycle;
  return out;
}
EngineEffects SimulationCore::MarkFailed(const std::string& message) {
  EngineEffects out;
  impl_->server_state = ERROR_PAUSED;
  impl_->status_message = message;
  for (auto& [id, c] : impl_->controllers) {
    auto old = c.state;
    c.state = ERROR;
    c.pending_policy = c.policy.error_policy;
    c.command_dirty = false;
    out.adapter_changes.push_back(Change(AdapterChangeKind::kServerFailed, id, CONTROLLER,
                                         c.session, old, ERROR, impl_->current_tick, message));
  }
  for (auto& [id, o] : impl_->observers) {
    auto old = o.state;
    o.state = ERROR;
    out.adapter_changes.push_back(Change(AdapterChangeKind::kServerFailed, id, OBSERVER, o.session,
                                         old, ERROR, impl_->current_tick, message));
  }
  out.status_urgency = EngineEffects::StatusUrgency::kTerminal;
  return out;
}
EngineEffects SimulationCore::MarkStopped(const std::string& message) {
  impl_->server_state = STOPPED;
  impl_->status_message = message;
  EngineEffects out;
  out.status_urgency = EngineEffects::StatusUrgency::kTerminal;
  return out;
}
ServerStatus SimulationCore::MakeStatus() const {
  ServerStatus out;
  out.epoch = impl_->epoch;
  out.tick_id = impl_->current_tick;
  out.simulation_time_s = impl_->model->simulation_time();
  out.state = impl_->server_state;
  out.message = impl_->status_message;
  for (const auto& [id, c] : impl_->controllers)
    out.adapters.push_back({id, CONTROLLER, c.session, c.state, c.message, c.state_since_tick,
                            c.timeout_count, c.last_command_sequence, 0, std::nullopt});
  for (const auto& [id, o] : impl_->observers)
    out.adapters.push_back({id, OBSERVER, o.session, o.state, o.message, o.state_since_tick,
                            o.timeout_count, 0, o.sequence, o.last_delivery_tick});
  std::sort(out.adapters.begin(), out.adapters.end(),
            [](const auto& a, const auto& b) { return a.adapter_id < b.adapter_id; });
  return out;
}
CoreState SimulationCore::state() const {
  CoreState out;
  out.epoch = impl_->epoch;
  out.current_tick_id = impl_->current_tick;
  out.simulation_time_s = impl_->model->simulation_time();
  out.timestep_s = impl_->model->timestep();
  out.server_state = impl_->server_state;
  for (const auto& [id, c] : impl_->controllers) {
    ++out.controller_counts[static_cast<std::size_t>(c.state)];
    if (c.state == ACTIVE_NO_COMMAND || c.state == ONLINE || c.state == OFFLINE)
      ++out.active_controller_count;
  }
  for (const auto& [id, o] : impl_->observers)
    ++out.observer_counts[static_cast<std::size_t>(o.state)];
  return out;
}
}  // namespace mujoco_sim::core
