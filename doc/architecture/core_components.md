# Core components

`SimulationCore` is the public V2 domain facade. It owns no transport objects and serializes no eCAL traffic. Its internal components have these boundaries:

- `ModelRuntime`: prepared model loading, the authoritative `mjModel`/`mjData`, control writes, and the only `mj_step()` call.
- `ResourceCatalog`: stable resource discovery, registration layouts, and canonical layout hashes.
- `AdapterRegistry`: the single Controller/Observer identity space, sessions, actuator ownership, heartbeat data, and replacement state.
- `ControllerStateMachine`: the explicit internal Controller phase rules. Wire states are projections and are never used as the stored domain state.
- `BarrierCoordinator`: the required/recipient snapshot and validated pending commands for one Tick.
- `TickStore`: immutable shared Tick handles, cached serialized bytes, identity lookup, and invalidation.
- `ObservationScheduler`: every-N-step and phase-locked simulation-time deliveries.
- `SnapshotManager`: isolated MuJoCo copies for viewers and sensor workers.
- `StatusProjection`: read-only construction of unified Adapter status.

All operations that can execute a step return an owned `StepCompletedEvent`. The event contains its source and observer frames, so the host never drains hidden Core queues. A Tick handle remains readable after invalidation, while `current` and identity lookup reject further publication.

Every mutating facade operation returns `EngineEffects`, either directly or in
an `EngineResult<T>`. Adapter lifecycle changes are typed with
`AdapterChangeKind`; the Server must not infer replacement, timeout, recovery,
or local error from response text. `status_changed` tells the owner when to
publish a new status projection.

Model replacement is prepared in this order: load `mjModel`, allocate and
initialize `mjData`, build the complete resource catalog, then commit all three.
Any validation failure before commit leaves the prior model, sessions, epoch,
Tick handles, and snapshots unchanged.

## Controller lifecycle

```text
Provisional --Activate--> Active --deadline--> TimedOutAwaitingProof
                              |                        |
                              | local error            | valid issued-Tick proof
                              v                        v
                            Error              RecoveryPending
                                                       |
                                                       | next Tick opens
                                                       v
                                                     Active
```

`TimedOutAwaitingProof` and `RecoveryPending` both project to Protocol V2
`TIMED_OUT`; the recovery-pending distinction is intentionally internal. A
recovery Commit validates identity, issued Tick range, status, and complete
command shape, but never enters the current Barrier and never applies its
actuator values. Opening the next Tick performs the transition to `Active`.

Observer phases are `Provisional`, `Active`, `TimedOut`, and `Error`, projected
directly to their corresponding V2 states. Observers never join a Barrier.

The only public lifecycle is `RegisterAdapter`, `ActivateAdapter`, and
`UnregisterAdapter`. There are no Controller-specific public wrappers. See
[controller_lifecycle.md](controller_lifecycle.md) for the complete Controller
and Observer transition rules and
[tick_commit_sequence.md](tick_commit_sequence.md) for Barrier sequencing.

## Public facade boundary

`SimulationCore` exposes 20 operational methods. Constructor, destructor,
deleted copy operations, and the move operations that transfer its single
Pimpl are lifecycle declarations rather than domain operations. Scalar queries
are grouped into one by-value `CoreState`; model resource queries are grouped
into one borrowed `CoreResources` view. This meets the refactor plan's initial
public-method target without combining unrelated mutations into a generic
command method.

The public facade intentionally continues to accept and return Protocol V2
messages so existing Server and test integrations retain wire semantics.
Internally, lifecycle phases, transitions, commit receipts, effects, Tick
identity, and step completion are typed domain values. Further removal of
Protobuf storage from adapter records is deferred to the plan's protocol
isolation phase; V2 field numbers and meanings remain unchanged.
