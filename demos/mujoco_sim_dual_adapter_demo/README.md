# Dual Adapter Demo

This package runs the existing WBR LQR Controller and double inverted pendulum
Controller against one `mujoco-sim-server`. It adds no C++ implementation. The
package composes installed adapter executables, strict YAML configuration, one
MuJoCo scene, and a bounded Bash process launcher.

The combined model uses MuJoCo model assets and body attachment. WBR resource
names remain unchanged, so the WBR session owns its six actuator commands. The
double inverted pendulum keeps its two actuator names and owns those commands
through a separate Controller session. Both sessions receive only the frozen
observation layout requested by their existing adapter.

The package copies MJCF runtime assets from its installed demo dependencies
during installation. This keeps an isolated colcon installation self-contained
without committing another copy of the vendored WBR meshes. The installed
`WBR_UPSTREAM.md` records their provenance.

Build from the repository root:

```bash
colcon build --packages-up-to mujoco_sim_dual_adapter_demo \
  --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

Start both Controllers and a GUI Server:

```bash
mujoco-sim-dual-adapter-demo-launch --mode gui
```

To run the three processes separately, source the workspace in each terminal
and use the same explicit Server name, `dual_adapter_manual`.

Terminal 1 — Server:

```bash
source install/setup.bash
mujoco-sim-server \
  --config install/mujoco_sim_dual_adapter_demo/share/\
mujoco_sim_dual_adapter_demo/config/dual_adapter_server.yaml \
  --server-name dual_adapter_manual
```

Terminal 2 — WBR LQR adapter:

```bash
source install/setup.bash
mujoco-sim-wbr-lqr-adapter \
  --config install/mujoco_sim_dual_adapter_demo/share/\
mujoco_sim_dual_adapter_demo/config/wbr_lqr_adapter.yaml \
  --server-name dual_adapter_manual \
  --adapter-id wbr_lqr_controller
```

Terminal 3 — double inverted pendulum adapter:

```bash
source install/setup.bash
mujoco-sim-double-inverted-control-demo \
  --config install/mujoco_sim_dual_adapter_demo/share/\
mujoco_sim_dual_adapter_demo/config/double_inverted_adapter.yaml \
  --server-name dual_adapter_manual \
  --adapter-id double_inverted_controller
```

Start the Server first. The two adapters may then be started in either order.
Stop all three processes separately with Ctrl-C.

Use `--mode headless` when no display is available. The default run is
interactive and continues until interrupted. A bounded acceptance run can use
the WBR adapter's State-sample budget:

```bash
mujoco-sim-dual-adapter-demo-launch \
  --mode headless \
  --wbr-ticks 6000 \
  --timeout-s 45 \
  --output-dir /tmp/mujoco_sim_dual_adapter
```

The launcher creates an isolated Server namespace by default. It waits for the
Server to become ready, starts both adapters, monitors all three processes, and
stops the remaining adapter and Server after a bounded run. Logs are retained
under `<output-dir>/logs`.

Configuration files may be overridden with `--server-config`, `--wbr-config`,
`--double-config`, and `--scene`. Binary overrides are available for test and
development builds. Run the launcher with `--help` for the complete interface.

Test the package:

```bash
colcon test --packages-select mujoco_sim_dual_adapter_demo \
  --event-handlers console_cohesion+
colcon test-result --all --verbose
```

The tests cover launcher cleanup with fake children, a 6000-State headless run
that requires WBR to finish in `NORMAL` while the pendulum adapter remains
active, and an Xvfb-backed GUI run when the Server was built with GUI support.
