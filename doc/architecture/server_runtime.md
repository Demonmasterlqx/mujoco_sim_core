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
terminal result when both inputs are pending at the same Step boundary. The
loop then calls `CheckDeadlines()` and performs at most one `Step()`. Completed
steps schedule Sensor capture, honor blocking Sensor fences, publish due State
and Observation frames, attempt a viewer snapshot capture, and publish
metrics/status. Viewer capture follows a steady-clock
`viewer_sync_period_ms` cadence. The loop uses `try_lock` to acquire and submit
a production lease; contention, closure, or buffer exhaustion skips that frame
without waiting.
Each Sensor worker exclusively calls its own eCAL publishers for subscriber
sampling and frame sends. The simulation loop requests periodic sampling
without waiting; a disconnect or reconnection may therefore affect capture on
the following Step rather than the requesting Step.

The viewer mailbox retains at most one pending production snapshot. A new
submission replaces an older pending snapshot, and the worker returns the
replaced buffer immediately. After taking a pending buffer, the GUI sync worker
releases the mailbox lock, waits for the MuJoCo viewer mutex if necessary,
copies into the stable display snapshot, and calls `Simulate::Sync()`. The
render thread consumes only MuJoCo's passive copy. GUI edits can affect the
stable display copy until the next sync but never reach live Core state.
Synchronization counts and durations are reset as one mutex-protected record.
If summary collection loses the non-blocking `try_lock`, the complete sync
record is retained for the next interval instead of being split across two
summaries.

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
