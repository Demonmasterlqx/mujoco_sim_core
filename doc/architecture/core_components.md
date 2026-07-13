# Core components

`SimulationCore` is the public V2 domain facade. It owns no transport objects and serializes no eCAL traffic. Its internal `detail` components have these boundaries:

- `ModelState`: transactional model loading, the authoritative `mjModel`/`mjData`, control writes, and the only `mj_step()` call.
- `ResourceCatalog`: stable resource discovery, registration layouts, and canonical layout hashes.
- `AdapterRegistry`: the single Controller/Observer identity space, sessions, activation, ownership, heartbeat, timeout, and replacement state.
- `BarrierCoordinator`: the required/recipient snapshot and validated pending commands for one Tick.
- `TickStore`: immutable shared Tick handles, cached serialized bytes, identity lookup, and invalidation.
- `ObservationScheduler`: every-N-step and phase-locked simulation-time deliveries.
- `SnapshotManager`: isolated MuJoCo copies for viewers and sensor workers.
- `StatusBuilder`: read-only construction of unified Adapter status.

All operations that can execute a step return an owned `StepCompletedEvent`. The event contains its source and observer frames, so the host never drains hidden Core queues. A Tick handle remains readable after invalidation, while `current` and identity lookup reject further publication.

The only lifecycle is `RegisterAdapter`, `ActivateAdapter`, and `UnregisterAdapter`. There are no Controller-specific wrappers.
