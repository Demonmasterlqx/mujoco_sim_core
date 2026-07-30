# Core components

`SimulationCore` owns MuJoCo and the asynchronous session records. Its internal
parts have deliberately narrow responsibilities:

- resource descriptors and canonical layout fingerprints;
- Controller records with ownership, lifecycle, granted policy, liveness,
  latest Command, pending policy write, and State cadence;
- Observer records with heartbeat, cadence, and bounded delivery metadata;
- completed-step generation for Controller State and Observation frames.

The Core does not own transport publishers, callback threads, wall-clock
pacing, or a generic event queue. Every liveness operation receives an explicit
`steady_clock` time so timeout behavior is deterministic in tests.

`SubmitCommand()` validates and replaces the session's latest Command.
`CheckDeadlines()` schedules offline/error/release policies and returns
lifecycle changes. `Step()` applies final Controller and policy writes, then
one-shot interactive actuator writes and an optional complete interactive body
wrench replacement, immediately before `mj_step()`. Interactive input neither
owns resources nor changes Adapter liveness. The Step advances tick and returns
due immutable frames. These operations never wait for a Controller or viewer.

Resource ownership is exclusive per actuator. Registration freezes dense
observation and command spans plus their 128-bit layout fingerprints. An
`OFFLINE` or `ERROR` session may be atomically replaced; unregister and expiry
release ownership at the same ordered Core boundary.
