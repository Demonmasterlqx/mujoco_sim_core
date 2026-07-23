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
- the simulation-loop lifecycle mutex protects the stop reason shared with
  callers of `RequestStop()`; the final exit code is atomic for `Join()`;
- the Controller Inbox mutex protects bounded per-session mailboxes and short
  registration prepare/commit sections. Registration reservations make
  `RequestClose()` reject new callback input without waiting; final `Close()`
  waits with the mutex released, so Core and transport setup run without an
  Inbox lock;
- Sensor worker locks protect their private capture queues, snapshot pools, and
  completion conditions. Each worker calls subscriber-count and send operations
  on its eCAL publishers only from its own thread and without that lock;
- the MuJoCo viewer mutex protects the viewer's copied model/data while the GUI
  thread renders and the simulation thread synchronizes its snapshot;
- logger and eCAL internals retain their own synchronization.

The Adapter SDK has separate client-side synchronization responsibilities:

- each client lifecycle gate admits one `Start`, `Activate`, cleanup, or other
  lifecycle operation through a short critical section. The admission lease
  contains no locked mutex, so RPC, eCAL publication, callback removal, callback
  drain, and thread join execute without the lifecycle mutex;
- Controller and Observer state mutexes protect session identity, phase,
  diagnostics, and their depth-one or bounded mailboxes. The Controller
  publisher and Observer heartbeat release these mutexes before transport calls;
- Sensor stream state is protected by one client mutex. A per-stream atomic
  callback flag rejects overlapping same-stream callbacks without blocking an
  eCAL receive thread; callbacks for different streams may run concurrently;
- each callback gate mutex protects admission, the borrowed target pointer, and
  active counts by thread. Teardown disables admission before removing eCAL
  callbacks, then waits only for callbacks on other threads, which permits a
  callback to destroy its own client without waiting for itself;
- the Observer heartbeat-thread mutex and Controller publisher ownership logic
  protect thread handles only. Handles are moved out before `join()`, and these
  locks are not nested with client state mutexes;
- the process-wide eCAL runtime mutex protects lease counts and
  initializing/finalizing transition state. It is released before invoking
  `eCAL::Initialize()` or `eCAL::Finalize()`; concurrent acquisition waits on a
  condition variable until that transition commits.

`Stop()` publishes atomic cancellation and wakes local conditions before it
waits for lifecycle admission. No Adapter SDK project mutex is held while
calling user callbacks, transport methods, eCAL callback removal, or thread
join. A failed remote unregister preserves session identity independently from
local terminal state so cleanup can be retried later.

No Server lock serializes Core calls, and no callback, Sensor worker, or GUI
render thread may call Core. The event queue mutex is released before an RPC
waits on its promise and before the loop processes an event. The lifecycle mutex
is not held while joining threads, waiting for Sensor capture, publishing eCAL
messages, or invoking Core.

`OffscreenRenderer::Create()` has one deliberate third-party exception. MuJoCo
caches its OpenGL dispatch table process-wide, so a process-global Sensor
platform mutex serializes the initial EGL/OSMesa family decision, native context
creation, and MuJoCo render-context initialization. The mutex must cover those
third-party calls until the selected platform family and retained dispatch
library are committed. Capture, publication, callbacks, and worker joins do not
use this mutex.

Published and queued values own their data. Parsed protobuf events own their
messages, completed-step results own immutable State and Observer frames, and
Sensor jobs own snapshots. Each Controller Inbox slot owns only its latest
decoded Command.

Blocking Sensor fences intentionally pause the sole Core writer after a step.
This means registry RPCs queued during the fence wait until it completes; they
do not run concurrently with Core. A stop request does not enter Core: it closes
the event queue and Inbox and signals the Sensor conditions, allowing the loop
to leave the fence and finish teardown.
