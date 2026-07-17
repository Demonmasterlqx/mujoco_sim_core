# WBR LQR Demo

This package integrates the upstream `CosmosMount/wbr_mujoco` controller with
the repository's asynchronous Protocol V4 runtime. The executable registers 21
observation resources and six actuator commands, activates one Controller
session, runs the upstream LQR state/control pipeline, and updates the latest
dense Command for each `StateLease`.

`StateDecoder` validates the registered layout and caches every observation
span in its constructor. `TorqueCommandMapper` caches command offsets and
ranges. The control loop therefore performs no resource-name lookup and does
not expose generated Protobuf types.

The interface configuration supports renamed and reordered MuJoCo resources.
The frozen Server layout, not lexical resource order, determines dense offsets.
Torque values are checked for finiteness and clamped to registered actuator
ranges before `SetCommand()`.

From the repository root after sourcing the installed workspace, start the
Server and Controller in separate terminals. Both configuration files are
strict, and CLI values override their YAML counterparts:

```bash
mujoco-sim-server --config \
  demos/mujoco_sim_wbr_lqr_demo/config/wbr_server.yaml \
  --server-name wbr --gui

mujoco-sim-wbr-lqr-adapter --config \
  demos/mujoco_sim_wbr_lqr_demo/config/wbr_lqr.yaml \
  --server-name wbr
```

The Server configuration resolves `../mjcf/scene.xml` relative to its own
directory and selects the `home` keyframe. Use `--headless` to override its GUI
default. The Controller exits after `adapter.max_ticks` State samples (`0`
means unlimited) and unregisters its session.

Build and test:

```bash
cmake -S demos/mujoco_sim_wbr_lqr_demo -B build/mujoco_sim_wbr_lqr_demo \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_wbr_lqr_demo -j
ctest --test-dir build/mujoco_sim_wbr_lqr_demo --output-on-failure
```

The suite covers default and renamed/reordered interfaces, dense decoding,
torque mapping, upstream model contract, installed consumption, headless eCAL,
and GUI execution. Upstream provenance is documented in [UPSTREAM.md](UPSTREAM.md)
and migration details in [MIGRATION_REPORT.md](MIGRATION_REPORT.md).
