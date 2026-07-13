# Locking model

The Server uses a single-writer ownership model rather than a mutex around
`SimulationCore`. After initialization, only the `SimulationLoop` thread may
call Core. eCAL callbacks communicate with it through `ServerEventQueue`; Sensor
workers and the GUI consume snapshots and never borrow the live `mjModel` or
`mjData`.

The remaining locks have narrow infrastructure responsibilities:

- the event queue mutex protects queue storage, byte accounting, close state,
  and its condition variable;
- the transport callback gate protects accepting/active callback counts so
  teardown can stop new callbacks and wait for current callbacks to return;
- the simulation-loop lifecycle mutex protects the stop reason, exit code, and
  completion state shared with callers of `RequestStop()` and `Join()`;
- Sensor worker locks protect their private capture queues, snapshot pools, and
  completion conditions;
- the MuJoCo viewer mutex protects the viewer's copied model/data while the GUI
  thread renders and the simulation thread synchronizes its snapshot;
- logger and eCAL internals retain their own synchronization.

No Server lock serializes Core calls, and no callback, Sensor worker, or GUI
render thread may call Core. The event queue mutex is released before an RPC
waits on its promise and before the loop processes an event. The lifecycle mutex
is not held while joining threads, waiting for Sensor capture, publishing eCAL
messages, or invoking Core.

Published and queued values own their data. Parsed protobuf events own their
messages, completed-step effects own Observer frames, Sensor jobs own snapshots,
and Tick publication uses `shared_ptr<const IssuedTick>`. Tick invalidation
marks an issued identity non-current without destroying an in-flight handle.

Blocking Sensor fences intentionally pause the sole Core writer after a step.
This means registry RPCs queued during the fence wait until it completes; they
do not run concurrently with Core. A stop request does not enter Core: it closes
the queue and signals the stop token and Sensor condition, allowing the loop to
leave the fence and finish teardown.
