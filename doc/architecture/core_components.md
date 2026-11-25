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

`SetPaused()` controls the normal `RUNNING`/`PAUSED` transition. `PAUSED` stops
only `Step()` from completing a physics step; it is distinct from terminal
`ERROR_PAUSED` and cannot be used to recover that state. Idempotent pause/run
requests are accepted without a state transition. Wall-clock pacing remains a
Server responsibility.

`Reset()` is accepted only in `RUNNING` or `PAUSED`. It calls
`mj_resetDataKeyframe()` for the configured `initial_keyframe`, or
`mj_resetData()` otherwise, zeros every actuator control, and runs
`mj_forward()`. Reset preserves the model epoch, completed-step tick, current
Server state and message, sessions, layouts, ownership, and all protocol
sequences. It discards pre-Reset queued policy writes, cached Controller
commands, and interactive wrenches, then schedules zero policy for active
Controller resources. The next completed Step retains monotonic tick identity
by using the previous tick plus one even when MuJoCo simulation time returns to
the configured initial value.

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

On Reset, an `ONLINE` Controller returns to `ACTIVE_NO_COMMAND`; other
Controller lifecycle states remain unchanged. Every non-provisional Controller
forgets its cached Command while preserving its last accepted Command sequence,
State sequence, session, layout, and ownership. Active command liveness restarts
at the Reset steady-clock time, while `OFFLINE` timing/statistics and `ERROR`
diagnostics remain intact. Observer sessions and sequences are preserved, but
their cadence is rebuilt so the first completed post-Reset Step is due.
