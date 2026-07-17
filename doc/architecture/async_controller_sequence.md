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
    L->>C: CheckDeadlines(steady_now)
    L->>C: Step()
    C->>C: apply final Command or policy write
    C->>M: mj_step()
    M-->>C: completed state
    C-->>L: due State and Observation frames
    L-->>A: publish latest State
    A->>A: compute independently
    A-->>T: publish latest Command
  end
```

Commands A, B, and C arriving before one Step collapse to C. If none arrives,
the last ctrl value remains. Publisher send failure does not stop either side:
the SDK retries identical sequence/bytes, and the Server continues stepping.

Realtime pacing performs at most one Step per iteration and reanchors a wall
deadline when more than one period late. Unthrottled pacing handles events and
Inbox once, then performs one Step without waiting.
