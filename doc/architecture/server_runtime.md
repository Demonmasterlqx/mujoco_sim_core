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

Output publishers and Inbox slots are created only for registered sessions.
Unregister, replacement, and offline expiry publish one `UNREGISTERED` status
tombstone, then remove the publisher and slot. Transport send failure is
non-terminal and never pauses simulation.

Shutdown closes ordinary events and the Inbox, completes queued RPC promises,
requests the loop stop token, interrupts Sensor fences, joins the loop, disables
callbacks, publishes terminal Server status, destroys transport, and finalizes
eCAL.
