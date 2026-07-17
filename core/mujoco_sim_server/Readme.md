# mujoco_sim_server

`mujoco-sim-server` composes MuJoCo Core, Protocol V4 eCAL transport, optional
Sensor capture, status tooling, and headless or GUI execution.

## Runtime

`SimulationLoop` is the only production caller of `SimulationCore`. Registry
and Observer heartbeat callbacks submit bounded typed events. Controller
Command callbacks check MJDATA framing and scalar decoding, while Fault
callbacks parse one bounded Protobuf message. Both update per-session depth-1
slots in `ControllerCommandInbox`; unknown sessions allocate no slot. The first
pending value wakes the loop without placing Controller payloads in the general
FIFO.

Each loop iteration handles ordinary events, drains Commands followed by
Faults, checks liveness, and executes at most one Step. Draining Faults last
ensures a valid Fault wins over Commands received for the same boundary.
`realtime` pacing uses a wall period of model timestep divided by
`realtime_factor` and reanchors when late; `unthrottled` pacing performs one
Step per iteration without a wall-clock wait. Missing Controllers, OFFLINE
sessions, missing subscribers, and publish failures never block Step.

Completed-step ordering remains Sensor snapshot/blocking fence, Controller
State and Observer publication, viewer snapshot, status, and metrics. The
Sensor blocking fence may pause the single writer after a Step, but external
Controller computation is never in that path.

## Configuration

The authoritative strict schema is
[`config/server_config.yaml`](config/server_config.yaml). The excerpt below
shows the Controller and pacing fields together with the essential runtime
identity fields. Relative paths, including `model_path`, are resolved from the
configuration file's directory:

```yaml
server_name: default
model_path: model.xml
pacing_mode: realtime
realtime_factor: 1.0
command_timeout_ms: 500
command_keepalive_ms: 100
offline_session_expire_ms: 0
controller_state_every_n_steps: 1
controller_state_period_s: 0.0
controller_state_max_rate_hz: 1000.0
startup_ctrl_policy: zero
offline_ctrl_policy: hold_last
error_ctrl_policy: zero
release_ctrl_policy: zero
controller_overrides: []
session_timeout_ms: 1000
observer_heartbeat_period_ms: 250
log_summary_period_ms: 1000
headless: true
```

Per-controller overrides are complete policy records keyed by unique
`adapter_id`. State cadence uses `period_s` when nonzero and otherwise
`every_n_steps`; grants are clamped to `controller_state_max_rate_hz`. The
generated parser rejects unknown fields, including all removed synchronous
Controller configuration.

Every override supplies the complete policy for that Adapter rather than
merging omitted values from the global Controller defaults:

```yaml
controller_overrides:
  - adapter_id: arm
    command_timeout_ms: 250
    command_keepalive_ms: 50
    offline_session_expire_ms: 5000
    state_every_n_steps: 2
    state_period_s: 0.0
    startup_ctrl_policy: zero
    offline_ctrl_policy: hold_last
    error_ctrl_policy: zero
    release_ctrl_policy: zero
```

Adapter IDs in this sequence must be unique. Timeout and keepalive must be
positive, with keepalive strictly shorter than timeout. Cadence step counts
must be positive, cadence periods must be finite and nonnegative, and policies
accept only `zero` or `hold_last`. A positive `state_period_s` selects
simulation-time cadence; otherwise `state_every_n_steps` applies. The global
maximum State rate can only slow the granted cadence.

Only identity, model selection, run limit, activation timeout, viewer mode,
and logging fields have direct CLI flags; pacing and Controller policy are YAML
fields. CLI values override their YAML counterparts. `--max-ticks 0` runs until
interruption or runtime shutdown.

```bash
mujoco-sim-server --config server.yaml --headless
mujoco-sim-status --server-name default --watch --json
```

Status is periodic and lifecycle changes are coalesced. Unregister, expiry, and
replacement publish one `UNREGISTERED` tombstone before the session publisher
and Inbox slot are removed. Terminal Server status is published before
transport teardown.

## Periodic Runtime Summary

At info level, the Server emits `event=server_summary` every
`log_summary_period_ms`. Set the period to `0` to disable these records. Each
record describes the interval since the preceding summary and includes:

- `tick_id`, `interval_steps`, `simulation_time_s`, and
  `interval_simulation_s`
- `interval_wall_s`, measured with the steady clock
- `step_rate_hz`, computed as `interval_steps / interval_wall_s`
- `realtime_factor_rate`, computed as
  `interval_simulation_s / interval_wall_s`
- `target_realtime_factor` and `pacing_mode`
- active/online/offline/error Controller counts, online Observer count,
  sampled transport endpoint count, and interval-local malformed-message count

`realtime_factor_rate` is the measured simulation advance rate, not the
configured target. In `realtime` mode it can be compared with
`target_realtime_factor`; in `unthrottled` mode it is throughput-dependent and
may be substantially higher. The malformed-message count is reset after each
summary. `active_controllers` counts `ACTIVE_NO_COMMAND`, `ONLINE`, and
`OFFLINE` sessions; terminal `ERROR` sessions are reported separately.
`transport_endpoints` is refreshed at
`transport_subscriber_sample_period_ms`; it is diagnostic only and never gates
simulation stepping or State publication.

The detailed loop boundary and ownership rules are documented in
[Server runtime](../../doc/architecture/server_runtime.md) and the
[Locking model](../../doc/architecture/locking_model.md).
