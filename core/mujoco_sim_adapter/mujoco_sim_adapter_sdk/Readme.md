# mujoco_sim_adapter_sdk

The Adapter SDK provides Protocol V4 clients for asynchronous Controllers,
Observers, and Sensor streams. Controller and Observer APIs expose domain DTOs,
immutable leases, spans, and dense scalar arrays. The Sensor API intentionally
exposes Protocol V4 `StreamDescriptor`, `StreamFrameHeader`, and discovery
Protobuf values together with owned or callback-scoped raw payload bytes.

Lifecycle operations are serialized by a short-lock admission gate. The gate's
mutex is released before RPC, transport publication, callback teardown, or
thread joins begin. `Stop()` publishes cancellation state before waiting for an
in-progress lifecycle operation, so bounded RPC loops and blocking reads can
observe shutdown without acquiring the lifecycle gate.

Sensor callbacks for one stream never overlap. If another frame arrives while
that stream's user callback is still running, the new frame is dropped and
included in `dropped_samples()` instead of blocking a transport thread or
building an unbounded callback backlog.

Every public call with a `std::string*` diagnostic requires a non-null pointer
and clears it on valid entry. Success and `kReceived`, `kTimeout`, or `kStopped`
results leave it empty; `false` and `kError` provide contextual diagnostics.
State, Observation, and Sensor sample outputs are transactional and change only
for `kReceived`. A client is single-use after `Stop()` or successful cleanup.

Server `PAUSED` is nonterminal. Controller State waits, Observer Observation
waits, and Sensor sample waits remain active; Controller command publication
and Observer heartbeat continue. Only normal `STOPPED` ends waits normally and
`ERROR_PAUSED` ends them with an error.

## Controller

```cpp
mujoco_sim::adapter_sdk::ControllerOptions options;
options.server_name = "default";
options.adapter_id = "arm";
options.observation_names = {"hinge_pos"};
options.command_names = {"hinge_motor"};

mujoco_sim::adapter_sdk::ControllerClient client(options);
std::string error;
if (!client.Start(&error) || !client.Activate(&error)) return 1;

const auto command = client.layout().Command("hinge_motor");
if (!command) return 2;
mujoco_sim::adapter_sdk::StateLease state;
if (client.WaitForNewState(std::chrono::milliseconds(100), &state, &error) ==
    mujoco_sim::adapter_sdk::StateReadResult::kReceived) {
  std::vector<double> values(client.layout().command_scalar_count());
  values[command->offset] = 0.25;
  if (!client.SetCommand(values, &error)) return 3;
}
if (!client.Unregister(&error)) return 4;
```

`ControllerLayout` resolves resource names to `{offset, count}` once after
registration. `StateLease` is move-only and owns an immutable mailbox buffer.
`ReadLatestState()` is non-blocking; `WaitForNewState()` waits for a newer
sequence than the last successful read. Both return `kReceived`, `kTimeout`,
`kStopped`, or `kError`; a successful read replaces the output lease. State
reads permit one user thread and `SetCommand()` permits one control thread.

`SetCommand()` validates exact scalar count, finite values, and registered
ranges before replacing the depth-1 latest-command mailbox. A publisher thread
sends each new generation and republishes the latest value at the granted
keepalive cadence. A plain transport rejection retains the same sequence and
bytes for retry; newer mailbox generations remain pending until that retry
succeeds. A structured serialization, endpoint, or eCAL error is terminal and
is returned by subsequent State reads instead of being retried indefinitely.
Controllers do not send session heartbeats.

A local Server Reset preserves the Controller session, layout, ownership, and
last accepted Command sequence, but changes an `ONLINE` session to
`ACTIVE_NO_COMMAND` and clears the Server's cached command. The SDK publisher
continues sending its latest value at keepalive cadence with increasing wire
sequences, so it naturally supplies a post-Reset Command. A custom Protocol V4
publisher must likewise send a sequence strictly greater than the last
pre-Reset Command; replaying an old sequence cannot reactivate the Controller.

The registration response supplies the Server-granted command timeout,
keepalive, State cadence, control policies, and offline expiry. `Stop()` wakes
waiters and stops local publishing without removing the remote session.
`Unregister()` performs remote cleanup, then stops local publishing and
reception; it preserves session identity after a failed RPC so the call can be
retried. Destruction attempts both local shutdown and best-effort unregister.
`ReportFault()` publishes one of `kControllerError`, `kInvalidOutput`, or
`kInternalControllerError`; an accepted fault makes the current Server session
terminal `ERROR`. Rejection or a structured transport failure is returned to
the `ReportFault()` caller with its contextual diagnostic. A received
`ERROR`/`UNREGISTERED` status, or disappearance of a session after it has
appeared in status, terminates subsequent State waits.

`Start()` registers and starts State/status reception but does not activate the
session. `Activate()` starts Server-side liveness and the Command publisher;
`SetCommand()` is rejected before activation. Activation does not synthesize
State: the first lease is produced by the next completed Step for which the
Server-granted cadence is due. If the Server accepts activation but local
cancellation or publisher-thread creation prevents local completion, the SDK
stops local reception and attempts a compensating unregister. A failed cleanup
keeps the session identity so `Unregister()` can retry it. A client is
single-use and cannot be restarted after `Stop()` or successful `Unregister()`.

## Observer and Sensor streams

Observers retain explicit cadence, heartbeat, delivery, and bounded mailbox
semantics. `ObserverClient::Start()` registers and begins reception and
heartbeat, `Activate()` transitions the provisional session to `ONLINE`, and
`Unregister()` provides retryable remote cleanup. `ObservationLease` includes
sequence, completed-step tick, simulation time, and dense observations.
An ordinary heartbeat rejection is recorded by `lifecycle_warning()` and
retried at the next cadence; serialization, missing-endpoint, and eCAL errors
are terminal and wake observation waiters with `kError`. A matching remote
`ERROR`, `OFFLINE`, or `UNREGISTERED` state, replacement of the session, or
disappearance after the session was first observed is also terminal. Wrong
epochs and unrelated IDs/roles before the first match are ignored. A normally
`STOPPED` Server wakes waiters with `kStopped` and no diagnostic.

Sensor clients do not create an Adapter session. They discover V4 stream
descriptors and receive raw payloads with the separate `MJSTREAM` 1.0 framing
and V4 `StreamFrameHeader` into per-stream bounded mailboxes, or deliver a
borrowed `StreamSampleView` synchronously to a callback. Sensor payloads do not
enter Controller State or Observer frames. The borrowed payload is valid only
until the callback returns; only `StreamSampleView::Copy()` creates a sample
that may be retained beyond that callback.

## Build and test

After building the workspace dependencies and sourcing `install/setup.bash`:

```bash
cmake -S core/mujoco_sim_adapter/mujoco_sim_adapter_sdk \
  -B build/mujoco_sim_adapter_sdk -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_adapter_sdk -j
ctest --test-dir build/mujoco_sim_adapter_sdk --output-on-failure
```

`MUJOCO_SIM_RUN_SERVER_INTEGRATION_TESTS=ON` is the default and adds the Sensor
runtime and lifecycle Server E2E tests. Set it to `OFF` only when Core/Server
packages are unavailable; SDK unit tests, real eCAL tests, strict documentation,
quality gates, and the installed-package consumer remain available. A strict
standalone SDK gate can be configured with:

```bash
cmake -S core/mujoco_sim_adapter/mujoco_sim_adapter_sdk \
  -B build/mujoco_sim_adapter_sdk-quality \
  -DBUILD_TESTING=ON \
  -DMUJOCO_SIM_RUN_SERVER_INTEGRATION_TESTS=OFF \
  -DMUJOCO_SIM_BUILD_DOCS=ON \
  -DMUJOCO_SIM_ENABLE_QUALITY_GATES=ON
cmake --build build/mujoco_sim_adapter_sdk-quality -j
cmake --build build/mujoco_sim_adapter_sdk-quality \
  --target mujoco_sim_adapter_sdk_docs
ctest --test-dir build/mujoco_sim_adapter_sdk-quality --output-on-failure
```
