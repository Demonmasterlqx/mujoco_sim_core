# Locking model

The Core mutex protects non-thread-safe `SimulationCore` calls and short snapshot copies. It never protects eCAL calls, protobuf serialization, logging, blocking sensor waits, or worker joins.

Endpoint publisher mutexes protect publisher maps and serialize creation, `Send()`, and removal for a publisher. They are never held while calling Core. The application mutex protects lifecycle state, stop reason, dirty state, scheduler notification, and `step_pipeline_busy`; it is not held during teardown joins.

Published data is owning data: Tick batches use `shared_ptr<const IssuedTick>`, completed-step events own observer frames, and status publication owns its protobuf snapshot. Invalidation removes the Tick identity and flips `current`, but does not destroy an in-flight handle.

Blocking sensor fences occur after releasing both Core and simulation pipeline execution locks. Heartbeat and status callbacks can therefore acquire the Core mutex while a sensor worker finishes.
