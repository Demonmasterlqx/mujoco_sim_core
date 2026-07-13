# Server runtime

`ServerApplication` is the composition root. It owns the Core, scheduler, four endpoint groups, sensor runtime, viewer runtime, logger, and runtime metrics.

`RegistryEndpoint` owns registry services and the unified heartbeat subscriber. `ControlEndpoint` owns Commit input and Tick publishers. `ObservationEndpoint` owns Observer publishers. `StatusEndpoint` owns status publication. Endpoint objects own their eCAL objects and perform serialization and `Send()` outside the Core lock.

`SimulationScheduler` controls wall-clock deadlines, retransmission, empty barriers, status/viewer deadlines, and `max_ticks`. Its explicit `step_pipeline_busy` gate prevents opening the next Tick until post-step work finishes without extending the Core critical section.

For every completion source, the application follows one pipeline: obtain the event under the Core lock, enqueue the required snapshot, release the lock, wait for blocking sensors, publish observer frames, update metrics, clear the pipeline gate, and wake the scheduler.

Shutdown stops new callbacks, drains active callbacks, stops workers, destroys endpoints, and then releases the eCAL runtime.
