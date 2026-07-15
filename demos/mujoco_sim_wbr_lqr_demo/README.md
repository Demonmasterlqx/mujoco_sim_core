# WBR LQR Demo

This package integrates the upstream `CosmosMount/wbr_mujoco` controller with
the repository's Protocol V3 lockstep runtime. The executable registers 21
observation resources and six actuator commands, activates one synchronous
Controller session, runs the upstream LQR state/control pipeline, and commits a
dense command vector for each `TickLease`.

`TickDecoder` validates the registered layout and caches every observation
span in its constructor. `TorqueCommandMapper` caches command offsets and
ranges. The control loop therefore performs no resource-name lookup and does
not expose generated Protobuf types.

The interface configuration supports renamed and reordered MuJoCo resources.
The frozen Server layout, not lexical resource order, determines dense offsets.
Torque values are checked for finiteness and clamped to registered actuator
ranges before Commit.

Run from an installed workspace:

```bash
mujoco-sim-server \
  --model demos/mujoco_sim_wbr_lqr_demo/mjcf/wbr_scene.xml \
  --server-name wbr --gui

mujoco-sim-wbr-lqr-demo --server-name wbr
```

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
