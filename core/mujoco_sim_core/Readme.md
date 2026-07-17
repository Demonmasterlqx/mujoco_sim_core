# mujoco_sim_core

`SimulationCore` owns the live MuJoCo model/data, resource ownership, Controller
and Observer sessions, command liveness, and completed-step frame production.
Production code calls it only from `SimulationLoop`, preserving a single writer.

The package is transport-independent: it contains no eCAL publisher,
subscriber, RPC service, command queue, or pacing thread. Callers serialize
all mutating operations and pass explicit `std::chrono::steady_clock` times to
command and deadline APIs when deterministic liveness behavior matters.

## Asynchronous Controller model

The public runtime operations are `SubmitCommand()`, `Step()`, and
`CheckDeadlines()`. `SubmitCommand()` validates identity, epoch, layout,
strictly increasing session sequence, finite values, and actuator ranges, then
stores only the latest Command. It never steps the model.

At each `Step()` boundary Core applies the final pending policy/Command write,
calls `mj_step()`, increments the completed-step tick, and creates due
Controller State and Observer frames. Several Commands received before one
boundary collapse to the newest. With no new Command, MuJoCo ctrl retains its
last value.

Controller State cadence is anchored at activation and may use
`every_n_steps` or simulation-time `period_s`. Activation emits no initial
State. `OFFLINE` sessions continue receiving State; `ERROR` sessions do not.
Controller State sequence is per session and starts at one. Observer delivery
and heartbeat semantics remain independent of Controller command liveness;
Controllers do not send session heartbeats.

## Lifecycle and policy

Controller states are `PROVISIONAL`, `ACTIVE_NO_COMMAND`, `ONLINE`, `OFFLINE`,
`ERROR`, and `UNREGISTERED`. Command liveness begins at activation. The first
valid Command enters `ONLINE`; timeout enters `OFFLINE`; a later valid Command
recovers immediately. Invalid, stale, and duplicate Commands are rejected
without refreshing liveness or entering `ERROR`. A valid reported Fault makes
the current session terminal `ERROR`.

Startup, offline, error, and release policies are independently `zero` or
`hold_last`. Policy writes take effect at the next Step boundary, and a later
Command at that boundary wins. Optional offline expiry unregisters the session,
releases ownership, and applies release policy.

Replacing an `OFFLINE` or `ERROR` session with the same adapter ID is atomic.
Other existing states return `ALREADY_EXISTS`. Unregister, expiry, and
replacement surface an `UNREGISTERED` change so the Server can publish one
tombstone before removing transport resources.

## Core API boundaries

- `LoadModel()` creates a new model epoch and resource catalog.
- `RegisterAdapter()`, `ActivateAdapter()`, and `UnregisterAdapter()` manage
  exact session identities and exclusive command-resource ownership.
- `SubmitCommand()` only updates the latest accepted command; it never calls
  `mj_step()` and rejected commands do not refresh liveness.
- `Step()` is the only normal physics advance and returns completed-step
  Controller/Observer frames through `EngineEffects`.
- `CheckDeadlines()` performs activation expiry, Observer heartbeat timeout,
  Controller command timeout, and optional offline-session expiry without
  advancing physics.
- `CreateSnapshot()`, `CloneSnapshot()`, and `UpdateSnapshot()` provide
  isolated model/data copies for viewers and sensor workers.

`EngineEffects` is the sole side-effect description returned to the Server.
Transport publication happens after the Core operation; a publication failure
does not roll physics back or make a Controller part of the Step path.

## Build and test

From the repository root, after dependencies are available:

```bash
cmake -S core/mujoco_sim_core -B build/mujoco_sim_core \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_core -j
ctest --test-dir build/mujoco_sim_core --output-on-failure
```

The bundled MuJoCo distribution is used by default. Set
`-DMUJOCO_SIM_USE_BUNDLED_MUJOCO=OFF` to require an external MuJoCo CMake
package. Installed consumers use `find_package(mujoco_sim_core CONFIG
REQUIRED)` and link `mujoco_sim::core`.

See [Core components](../../doc/architecture/core_components.md),
[Controller lifecycle](../../doc/architecture/controller_lifecycle.md), and
[async sequence](../../doc/architecture/async_controller_sequence.md).
