# mujoco_sim_sample_adapter

This package is the minimal Protocol V3 Controller application built on
`mujoco_sim_adapter_sdk`. It registers requested resources, caches command
spans from `ControllerLayout`, waits for `TickLease`, fills one dense command
vector, commits, and unregisters.

Configuration is strict and generated from
[`config/sample_adapter.yaml`](config/sample_adapter.yaml). The relevant fields
are:

```yaml
server_name: default
adapter_id: sample_adapter
observation_names: [hinge_pos]
command_names: [hinge_motor]
command_value: 0.0
commit_timeout_ms: 1000
session_timeout_ms: 1000
heartbeat_period_ms: 250
service_timeout_ms: 5000
```

Unknown fields and removed field names are errors; there are no configuration
aliases. `commit_timeout_ms` is the per-Tick deadline, while
`session_timeout_ms` is the absence-of-presence deadline.

```bash
mujoco-sim-sample-adapter \
  --server-name default \
  --adapter-id sample_adapter \
  --observation hinge_pos \
  --command hinge_motor \
  --command-value 0.25
```

Build and test:

```bash
cmake -S core/mujoco_sim_adapter/mujoco_sim_sample_adapter \
  -B build/mujoco_sim_sample_adapter -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_sample_adapter -j
ctest --test-dir build/mujoco_sim_sample_adapter --output-on-failure
```

The suite includes unit, installed-consumer, headless eCAL, and GUI eCAL tests.
