# mujoco_sim_adapter_sdk

The Adapter SDK provides single-session Protocol V3 clients for synchronous
Controllers, asynchronous Observers, and Sensor streams. Public Controller and
Observer APIs expose domain DTOs, leases, spans, and dense scalar arrays; they
do not expose generated Protobuf types.

## Controller

```cpp
mujoco_sim::adapter_sdk::AdapterOptions options;
options.server_name = "default";
options.adapter_id = "arm";
options.observation_names = {"hinge_pos"};
options.command_names = {"hinge_motor"};
options.commit_timeout_ms = 1000;
options.session_timeout_ms = 1000;
options.heartbeat_period_ms = 250;

mujoco_sim::adapter_sdk::SyncAdapterClient client(options);
std::string error;
if (!client.Start(&error) || !client.Activate(&error)) return 1;

const auto command = client.layout().Command("hinge_motor");
mujoco_sim::adapter_sdk::TickLease tick;
if (client.WaitForTick(std::chrono::milliseconds(100), &tick, &error) ==
    mujoco_sim::adapter_sdk::TickWaitResult::kReceived) {
  std::vector<double> commands(client.layout().command_scalar_count());
  commands[command->offset] = 0.25;
  if (!client.Commit(tick, commands, &error)) return 2;
}
client.Unregister(&error);
```

`ControllerLayout` resolves names to `{offset, count}` once after registration.
Applications should cache those spans and use `TickLease::observations()` on
the hot path. `TickLease` is move-only and keeps its mailbox buffer alive.
`Commit()` verifies lease ownership, exact count, finite values, and registered
command ranges before encoding a dense Commit.

Duplicate Tick handling is internal. An uncommitted duplicate is not delivered
again; after Commit, the exact cached Commit bytes may be resent. A duplicate
identity with different Tick payload is terminal.

## Observer

`ObserverOptions` defaults to an explicit `period_s = 0.01` (100 Hz), queue
capacity 1, `session_timeout_ms = 1000`, and `heartbeat_period_ms = 250`.
Exactly one of `period_s` and `every_n_steps` must be set. The Server rejects a
cadence over its configured cap and rejects every-step delivery unless it is
explicitly enabled.

`WaitForObservation()` returns a move-only `ObservationLease` with sequence,
Tick, simulation time, and a dense observation span. Queue overflow drops the
oldest frame and contributes to `dropped_frames()` along with transport
sequence gaps. Observer heartbeat is periodic and never emitted per frame.

## Liveness and cleanup

Controller heartbeat is suppressed while Commit publishes are active. It is
sent only when both the last successful Commit publish and the last heartbeat
are at least `heartbeat_period_ms` old. Commit refreshes Server presence;
heartbeat cannot extend a Barrier deadline. Observer heartbeat runs at the
configured low rate.

`Stop()` performs local cancellation only: it wakes waiters, stops heartbeat,
and disables callbacks. `Unregister()` performs remote cleanup. A failed
Unregister preserves session identity so it can be retried. Callback teardown
disables delivery, removes callbacks, waits for in-flight leases, and then
destroys endpoints. Only one waiter is allowed per client.

## Sensor streams

`SensorStreamClient` discovers V3 stream descriptors and receives framed raw
payloads into bounded per-stream mailboxes. `OwnedStreamSample` owns both its
V3 `StreamFrameHeader` and payload. Stream data is not part of a Controller
Tick or Observer frame.

## Build and test

```bash
cmake -S core/mujoco_sim_adapter/mujoco_sim_adapter_sdk \
  -B build/mujoco_sim_adapter_sdk -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_adapter_sdk -j
ctest --test-dir build/mujoco_sim_adapter_sdk --output-on-failure
```

Tests cover lifecycle retry, callback teardown, single waiters, dense
encode/decode, duplicate behavior, heartbeat suppression/stop, mailbox
eviction, Sensor framing, installed consumption, and real eCAL E2E paths.
