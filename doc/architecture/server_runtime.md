# Server runtime

`ServerApplication` validates configuration, initializes eCAL and the optional
viewer, loads the model, and starts `SimulationLoop`. The loop owns
`SimulationCore`, Sensor runtime, pacing state, metrics, and viewer snapshots;
it is the only thread allowed to call Core.

Registry RPC and Observer heartbeat callbacks parse one bounded message and
enqueue a typed event. Controller Command and Fault callbacks parse the fixed
V4 payload and update separate depth-1 slots for a registered session. A burst
therefore consumes constant memory and cannot fill the ordinary event FIFO.

Each iteration drains a bounded ordinary-event batch, all pending Command slots,
then all pending Fault slots. Faults are drained last so a valid Fault is the
terminal result when both inputs are pending at the same Step boundary. The
loop then calls `CheckDeadlines()` and performs at most one `Step()`. Completed
steps schedule Sensor capture, honor blocking Sensor fences, publish due State
and Observation frames, update the viewer snapshot, and publish metrics/status.
Each Sensor worker exclusively calls its own eCAL publishers for subscriber
sampling and frame sends. The simulation loop requests periodic sampling
without waiting; a disconnect or reconnection may therefore affect capture on
the following Step rather than the requesting Step.

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

Shutdown first records the stop request, closes ordinary events, marks the
Inbox closed without waiting, completes queued RPC promises, and interrupts
Sensor fences. Sensor workers discard captures that have not started and join
only their active capture. The loop then drains registration reservations,
stops those workers, marks Core stopped, and publishes terminal Server status.
The calling thread joins the loop, disables and drains transport callbacks,
destroys transport resources, and finally finalizes eCAL when the Server owns
the process runtime.
