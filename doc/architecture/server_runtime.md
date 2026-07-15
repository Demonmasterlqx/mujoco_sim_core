# Server runtime

`ServerApplication` is the composition root. `Start()` validates configuration,
initializes eCAL and the optional viewer, loads the model, creates the transport,
and starts the simulation thread. `Run()` only waits for that thread in headless
mode; in GUI mode it runs rendering on the calling thread and then waits for the
simulation thread.

## Single-writer event loop

`SimulationLoop` owns `SimulationCore`, `SensorRuntime`, scheduling state,
runtime metrics, and the viewer snapshot. It is the only component allowed to
call `SimulationCore`, including read-style projections such as status and
resource queries. This makes all Core access ordered by one thread without a
Core mutex.

eCAL callbacks are transport boundaries only. They validate the 4 MiB protocol
payload limit, parse Protobuf control messages or fixed binary data frames
once, and submit a typed `ServerEvent`. Registry
RPC events carry a promise and wait for the loop's response. Commit and
Heartbeat topics are fire-and-forget events.

`ServerEventQueue` is FIFO and bounded to 1024 events and 64 MiB of parsed
payload. The loop consumes at most 64 events per batch before running
deadline, Tick, status, viewer, and metrics work. A registry RPC rejected by a
full or closed queue receives `INTERNAL_ERROR`; Commit and Heartbeat events are
dropped and counted. These bounds prevent callback traffic from creating
unbounded memory use or permanently starving timers.

All operations that may complete a step produce effects consumed by the same
`ApplyEffects()` path. Completed steps schedule Sensor capture, wait for any
blocking Sensor work, publish Observer frames, update metrics, and establish the
next Tick deadline. `SimulationLoop` encodes each Tick issue generation once,
caches the immutable bytes, and reuses them for at most three per-controller
retries while the issued identity is still current.

Status publishes every 100 ms in steady state. Lifecycle changes are coalesced
behind a 20 ms minimum interval, while terminal status is sent immediately
before teardown. Transport subscriber counts are sampled at 1 Hz; send paths
never query them.

## Transport and optional runtimes

`EcalServerTransport` owns eCAL services, subscribers, and publishers. Its input
callbacks never access Core. Output methods are called by the simulation loop,
so publisher maps also have a single owner and require no publisher mutex.

Sensor workers operate only on owning snapshots. A blocking stream pauses the
simulation loop after a completed step, preserving Sensor and Observation
ordering, but it does not grant a second thread access to Core. Stop requests
interrupt the blocking wait. The GUI render loop uses the viewer's own mutex and
an isolated snapshot synchronized by the simulation loop.

## Shutdown

`RequestStop()` is idempotent and may be called concurrently. The first request
records the reason, closes the event queue, completes queued RPC promises with
`INTERNAL_ERROR`, requests the simulation thread's stop token, wakes Sensor
waits, and asks the viewer to close. Shutdown then follows this order:

1. join the simulation loop;
2. stop accepting eCAL input and wait for active callbacks;
3. destroy eCAL transport objects;
4. destroy the loop, Sensor runtime, and viewer;
5. finalize eCAL if the application initialized it.

This order ensures no callback can enqueue into destroyed state and no RPC
caller remains blocked during teardown.
