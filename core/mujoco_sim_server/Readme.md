# mujoco_sim_server

`mujoco-sim-server` composes MuJoCo Core, Protocol V3 eCAL transport, optional
Sensor capture, status tooling, and headless or GUI execution.

## Runtime architecture

`SimulationLoop` is the only production caller of `SimulationCore` after model
load. eCAL callbacks enforce the 4 MiB limit, parse and structurally validate
one message, and enqueue a typed event into the bounded queue. They never read
Core state. The queue is limited to 1024 events, 64 MiB of actual payload, and
64 events per loop batch.

All five step sources (Commit, Deadline, Unregister, LocalError, EmptyBarrier)
pass through `ApplyEffects()` and `ApplyCompletedStep()`. Completed-step order
is Sensor snapshot/fence, Observer publish, metrics, then the next Tick time.

## Protocol and rates

The registry is `mujoco_sim.v3.<server>.registry`. Tick, Commit, and Observation
use the fixed 80-byte V3 dense frame; control/status messages use V3 Protobuf.
There are no older endpoint or configuration aliases.

Each Tick issue generation is encoded once in `SimulationLoop`. Initial publish
and retries share the same immutable bytes. Retries are per Controller with
default delays 5, 10, and 20 ms, stop after Commit, and never schedule beyond
that Controller's Commit deadline.

Server status is periodic at 100 ms by default. Lifecycle bursts are coalesced
behind `status_min_interval_ms` (20 ms by default); terminal status is sent
immediately before teardown. Tick/Commit traffic does not trigger status.
Transport subscriber counts are sampled at 1 Hz. Sensor lazy-capture subscriber
counts are sampled every 200 ms. Publish and capture paths use cached counts.

Observer cadence is mandatory. `observer_max_rate_hz` defaults to 500 Hz and
over-cap registrations are rejected. Every-step delivery is disabled unless
`allow_every_step_observer` is explicitly true.

## Configuration

```yaml
server_name: default
model_path: model.xml
tick_timeout_ms: 1000
session_timeout_ms: 1000
activation_timeout_ms: 5000
controller_heartbeat_period_ms: 250
observer_heartbeat_period_ms: 250
tick_retry_initial_ms: 5
tick_retry_backoff_multiplier: 2.0
tick_retry_max_delay_ms: 20
tick_retry_max_attempts: 3
status_period_ms: 100
status_min_interval_ms: 20
observer_max_rate_hz: 500
allow_every_step_observer: false
transport_subscriber_sample_period_ms: 1000
sensor_subscriber_sample_period_ms: 200
log_summary_period_ms: 1000
headless: true
```

The generated parser is strict. It validates timing relationships, including
heartbeat period at most one third of session timeout and status minimum
interval no greater than status period.

Run:

```bash
mujoco-sim-server --config server.yaml --headless
mujoco-sim-status --server-name default --watch --json
```

## Observability

The one-second `tick_summary` includes step/RTF/barrier latency, Tick frames and
bytes, maximum encode and Commit validation time, Observation frames and bytes,
Controller/Observer heartbeat counts, status classes, subscriber queries,
retry counts, encoded-cache hit/miss/entry counts, malformed messages, queue
rejections, and queue high watermark. Per-Tick and per-Commit logs are trace
only.

The status tool prints human output or JSON/NDJSON and reports unified Adapter
and Sensor stream state.

## Shutdown

Stop closes the queue and completes pending RPC promises, requests the
simulation stop token, interrupts blocking Sensor fences, publishes terminal
status, removes callbacks, waits for in-flight callbacks, destroys transport,
and finalizes eCAL. No Core mutex or second Core owner is introduced.

## Build and test

```bash
cmake -S core/mujoco_sim_server -B build/mujoco_sim_server \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_server -j
ctest --test-dir build/mujoco_sim_server --output-on-failure
```

The package provides unit, queue, Sensor, lifecycle, logging, status-tool,
installed-consumer, GUI, sanitizer, warnings, and four-configuration build
matrix gates.
