# mujoco_sim_sample_adapter

This package is the minimal Protocol V4 Controller application. It registers
resources, caches command spans from `ControllerLayout`, consumes immutable
`StateLease` values, updates the latest dense Command, and unregisters.

The executable accepts a strict YAML file through `--config`; command-line
values override the file. Its generated schema is
[`config/sample_adapter.yaml`](config/sample_adapter.yaml):

```yaml
server_name: default
adapter_id: sample_controller
observation_names: [hinge_pos, hinge_vel]
command_names: [hinge_motor]
max_ticks: 0
command_value: 0.0
service_timeout_ms: 5000
```

Runtime timeout, keepalive, cadence, and control policies are granted by the
Server during registration. Unknown and removed fields are errors.

With a Server already running and exporting the requested resources:

```bash
mujoco-sim-sample-adapter \
  --server-name default \
  --adapter-id sample_controller \
  --observations hinge_pos hinge_vel \
  --commands hinge_motor \
  --command-value 0.25 \
  --ticks 1000
```

The same values may be placed in a YAML document and passed with
`--config adapter.yaml`. `--ticks 0` (the default) is unlimited; the process
then runs until terminal session state or external termination. It unregisters
on normal loop exit, while failed registration or activation returns a nonzero
exit status.
