# mujoco_sim_core

`mujoco_sim_core` is the transport-independent, single-writer MuJoCo lockstep
engine. Its public API uses only domain types from
`<mujoco_sim/core/domain.hpp>` and
`<mujoco_sim/core/simulation_core.hpp>`. The package does not depend on
Protobuf, `mujoco_sim_protocol`, or eCAL.

## Ownership

After model load, production calls to `SimulationCore` are made only by the
Server's `SimulationLoop`. Core has no mutex and no internal transport thread.
Viewer and Sensor workers consume isolated `SimulationSnapshot` objects; they
never access the live `mjData`.

Every mutating operation returns `EngineEffects`. Commit, Deadline,
Unregister, LocalError, and EmptyBarrier completion all produce the same
`StepCompletedEvent` and pass through the Server's one effects pipeline.

## Dense sessions

Registration freezes a `ResourceLayout` containing descriptors, dense scalar
spans, exact scalar counts, and a 128-bit fingerprint. Issued Tick frames and
Observer frames contain only identity plus dense `std::vector<double>` values.
`ControllerCommit` contains dense commands. Resource lookup, MuJoCo IDs, command
ranges, and write offsets are prepared at registration, so the step path does
not perform name lookup.

```cpp
mujoco_sim::core::SimulationCore core;
auto loaded = core.LoadModel("model.xml", {});
if (!loaded.response.loaded) return 1;

mujoco_sim::core::RegisterAdapterRequest request;
request.set_adapter_id("arm");
request.set_role(mujoco_sim::core::CONTROLLER);
request.add_observation_names("hinge_pos");
request.add_command_names("hinge_motor");
request.set_commit_timeout_ms(1000);
request.set_session_timeout_ms(1000);
auto registration = core.RegisterAdapter(request);
```

Tick identity is immutable and remains readable through its shared handle after
invalidation. `IsIssuedTickCurrent()` is the publication guard. Wire encoding
and encoded-byte caching belong to `SimulationLoop`, not Core.

## Lifecycle and timing

Controller phases are Provisional, Active, TimedOutAwaitingProof,
RecoveryPending, and Error. Observer sessions never join a Barrier. Session
presence and per-Tick Commit deadlines are independent: a valid Commit refreshes
presence, while heartbeat never satisfies or extends the current Barrier.

Observer delivery must select exactly one cadence (`every_n_steps` or
`period_s`). Post-step frames are emitted in `StepCompletedEvent` order.
Status changes use `StatusUrgency`: routine metrics are periodic-only,
lifecycle changes are coalescible, and terminal changes are immediate.

See [Core components](../../doc/architecture/core_components.md),
[lifecycle](../../doc/architecture/controller_lifecycle.md), and
[Tick/Commit sequence](../../doc/architecture/tick_commit_sequence.md).

## Build and test

```bash
cmake -S core/mujoco_sim_core -B build/mujoco_sim_core -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_core -j
ctest --test-dir build/mujoco_sim_core --output-on-failure
```

The tests cover registration layout, identity/count/range validation, duplicate
Commit semantics, all five completion sources, timeout/recovery, replacement,
zeroing, Observer scheduling, snapshots, and the 10,000-step benchmark.
