# Server runtime

`ServerApplication` validates configuration, initializes eCAL and the optional
viewer, loads the model, and starts `SimulationLoop`. The loop owns
`SimulationCore`, Sensor runtime, pacing state, and runtime metrics; it is the
only thread allowed to call Core. `SimViewer` owns one lifetime-stable display
snapshot and two reusable production snapshots created by that loop during
initialization. The display snapshot owns an independent `mjModel` copy; the
two production snapshots share a separate model copy and therefore cannot race
with passive-viewer edits.

Registry RPC and Observer heartbeat callbacks parse one bounded message and
enqueue a typed event. Controller Command and Fault callbacks parse the fixed
V4 payload and update separate depth-1 slots for a registered session. A burst
therefore consumes constant memory and cannot fill the ordinary event FIFO.

Each iteration drains a bounded ordinary-event batch, all pending Command slots,
then all pending Fault slots. Faults are drained last so a valid Fault is the
terminal result when both inputs are pending at the same Step boundary. It then
merges the viewer mailbox, applies Reset before the latest Pause/Run selection,
calls `CheckDeadlines()`, performs at most one `Step()` only when the final state
is `RUNNING`, and processes timers. This order makes a same-boundary Controller
Command or Fault precede Reset and makes the selected Pause/Run mode apply to
the possible Step. Completed steps schedule Sensor capture, honor blocking
Sensor fences, publish due State and Observation frames, attempt a viewer
snapshot capture, and publish metrics/status. Viewer capture follows a
steady-clock
`viewer_sync_period_ms` cadence. The loop uses `try_lock` to acquire and submit
a production lease; contention, closure, or buffer exhaustion skips that frame
without waiting.

`PAUSED` is a normal nonterminal Server state. While paused, the loop continues
to process RPCs, heartbeats, Controller Commands and Faults, liveness deadlines,
status publication, wall-clock Sensor subscriber sampling, viewer management,
summaries, and stop conditions;
it does not call `PrepareStep()` or `mj_step()`. The loop waits on the ordinary
poll cadence rather than spinning. Resuming sets the realtime pacing anchor to
the current steady-clock time, so the loop never catches up wall time accumulated
during a pause. Pending viewer Control and Wrench input remains available for
the first resumed Step.
Each Sensor worker exclusively calls its own eCAL publishers for subscriber
sampling and frame sends. The simulation loop requests periodic sampling
without waiting; a disconnect or reconnection may therefore affect capture on
the following Step rather than the requesting Step.

The viewer mailbox retains at most one pending production snapshot. A new
submission replaces an older pending snapshot, and the worker returns the
replaced buffer immediately. After taking a pending buffer, the GUI sync worker
releases the mailbox lock, waits for the first-party render boundary and then
the MuJoCo viewer mutex, copies into the stable display snapshot, and calls
`Simulate::Sync()`. The render boundary spans the complete MuJoCo `Render()`
call through a first-party GLFW adapter, preventing render from observing a
partially updated scene or pending UI flag. The render thread consumes only
MuJoCo's passive copy. GUI edits can affect the stable display copy until the
next sync. When enabled (the default), the worker
extracts only changed `ctrl` values and complete dynamic-body `xfrc_applied`
state after Sync, releases the MuJoCo mutex, and merges them into a separate
input mailbox. It never copies display state back into live Core. The
SimulationLoop try-consumes this mailbox after Controller input and passes the
whitelist to `Step()`; same-boundary GUI controls are therefore last, but
remain one-shot writes.

The separate viewer input mailbox also owns local simulation actions. Pause/Run
is depth-one latest desired state and Reset is an edge latch. A Reset barrier
clears already queued Control and Wrench data, rejects data from the same sync,
and preserves a same-batch Pause/Run selection. The barrier remains active after
the SimulationLoop consumes the edge, until that loop completes the accepted or
rejected action. Completion advances a viewer input generation, clears pending
pre-completion snapshots, and makes any already active old-generation sync drop
its data output; the latest Pause/Run selection remains independent. The sync
worker captures the
MuJoCo pending Reset flag before `Simulate::Sync()` can clear it. When Reset is
disabled, its UI item is explicitly disabled and any unexpected pending flag is
cleared before Sync so the passive display copy cannot reset locally. The first
usable sync locates Pause/Run by its data pointer and Reset by exact item
identity; a missing or ambiguous item is a `viewer_sync_failed` runtime error.
Rejected Pause/Run actions are corrected asynchronously in the viewer without
feeding the correction back as another action.

Control mailbox slots retain the last unconsumed value per actuator. Wrench
mailbox state is depth-one latest, and an empty frame explicitly clears prior
GUI forces. Nonempty frames carry a steady-clock generation time. If no fresh
frame arrives before `viewer_mouse_force_timeout_ms`, the simulation thread
clears the active GUI wrench before its next Step and emits one expiry warning
for that stale episode. Input locks never block the simulation thread.
Synchronization counts and durations are reset as one mutex-protected record.
If summary collection loses the non-blocking `try_lock`, the complete sync
record is retained for the next interval instead of being split across two
summaries.

An accepted Reset first resets Core, then clears persistent viewer Control and
Wrench state, invalidates pre-Reset Sensor work, and rebuilds summary, pacing,
viewer-capture, and Sensor cadence anchors. Reset preserves `RUNNING` or
`PAUSED`, tick, epoch, sessions, layouts, and ownership; MuJoCo simulation time
returns to the configured initial state. `max_ticks` therefore remains based on
the monotonic completed-step tick and cannot be bypassed by Reset.

Each Sensor worker tags jobs with an atomic Reset generation. Reset increments
that generation, restores simulation-time cadence to the Reset time, returns
queued FIFO/latest/blocking snapshots, releases a blocking fence, clears pending
drop accounting and last-capture tick, and wakes waiters without waiting for
capture, serialization, or transport. Workers check generation before and after
capture and before serialization and send. Work that has already entered the
external `send()` call may finish after Reset, but an obsolete generation cannot
start another publication or restore its old last-capture tick or diagnostics.
Stream sequences and cumulative capture/publish/drop/error/blocking metrics do
not roll back.

Output publishers and Inbox slots are created only for registered sessions.
The loop preallocates its session-metadata node before invoking Core. A
successful Core and Inbox registration installs that node without allocation,
then applies replacement effects; an impossible duplicate-session commit is a
fatal Server invariant failure rather than a recoverable RPC error.
Unregister, replacement, and offline expiry publish one `UNREGISTERED` status
tombstone, then remove the publisher and slot. Transport send failure is
non-terminal and never pauses simulation. A plain eCAL send rejection is an
expected no-subscriber state and remains silent; dense-frame encoding,
Protobuf serialization, and transport exceptions produce contextual,
rate-limited diagnostics.

Logging is a required Server service. A logging sink that cannot be opened or
written during startup makes `ServerApplication::Start()` fail. A sink or
flush failure detected after startup allows bounded shutdown to finish and
changes the final process exit code to 2.

Shutdown first records the stop request, closes ordinary events, closes the
Inbox and viewer submission path, wakes the GUI sync worker, requests window
exit, completes queued RPC promises, and interrupts
Sensor fences. Sensor workers discard captures that have not started and join
only their active capture. The loop then drains registration reservations,
stops those workers, marks Core stopped, and publishes terminal Server status.
The calling thread joins the loop, disables and drains transport callbacks,
and the main-thread render loop joins the GUI sync worker after MuJoCo finishes
its mutex-protected cleanup. It then destroys transport resources and finally
finalizes eCAL when the Server owns the process runtime. A sync-worker
exception is contained at its thread entry, closes the window, and makes the
Server return exit code 2.
