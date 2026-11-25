# Asynchronous Controller sequence

```mermaid
sequenceDiagram
  participant A as ControllerClient
  participant T as eCAL callbacks / Inbox
  participant L as SimulationLoop
  participant C as SimulationCore
  participant M as MuJoCo

  A->>L: Register + Activate
  loop every Server iteration
    L->>L: process bounded ordinary events
    L->>T: drain depth-1 Command slots
    T-->>L: latest Command per session
    L->>C: SubmitCommand(latest, steady_now)
    L->>L: try-consume and merge local Viewer input
    L->>C: CheckDeadlines(steady_now)
    L->>C: Step(interactive input)
    C->>C: apply final Command/policy, then GUI control and wrench replacement
    C->>M: mj_step()
    M-->>C: completed state
    C-->>L: due State and Observation frames
    L-->>A: publish latest State
    A->>A: compute independently
    A-->>T: publish latest Command
  end
```

Commands A, B, and C arriving before one Step collapse to C. If none arrives,
the last ctrl value remains. An ordinary eCAL send rejection does not stop
either side: the SDK retries identical sequence/bytes, newer local generations
remain pending, and the Server continues stepping. Serialization, missing
endpoint, or eCAL exceptions are structured publication errors: the SDK makes
the Controller terminal and wakes subsequent State reads with `kError` instead
of retrying corrupted or unpublishable data indefinitely.

Local GUI Control writes are independently merged per actuator until one Step
completes. They are applied after Controller writes at the same boundary, but
are not persistent overrides: a later Command becomes the next writer. Viewer
mouse wrench frames are complete state replacements and expire from their
steady-clock generation time when synchronization stops.

SDK lifecycle admission is held across the logical operation but does not hold
a mutex across activation RPC, callback teardown, or publisher join. `Stop()`
sets cancellation before waiting for admission. If remote activation succeeds
but local publisher startup cannot commit, the client drains local work and
attempts compensating unregister; a failed cleanup remains explicitly retryable.

Realtime pacing performs at most one Step per iteration and reanchors a wall
deadline when more than one period late. Unthrottled pacing handles events and
Inbox once, then performs one Step without waiting.
