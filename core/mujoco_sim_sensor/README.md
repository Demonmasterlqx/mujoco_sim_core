# mujoco_sim_sensor

`mujoco_sim_sensor` provides transport-independent offscreen RGB, depth,
segmentation, camera point-cloud, and ray LiDAR capture from isolated MuJoCo
snapshots. It does not own eCAL endpoints or Protocol messages.

The Server maps captured buffers into Protocol V3 Sensor stream frames. A
blocking stream preserves lossless completed-step ordering and is interruptible
by the Server stop token. Non-blocking streams use bounded latest/FIFO queues.
Subscriber state is sampled by `SensorRuntime` every 200 ms by default, so
capture never performs a subscriber query and a newly connected consumer may
wait up to one sample period for lazy capture to begin.

```bash
cmake -S core/mujoco_sim_sensor -B build/mujoco_sim_sensor \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_sensor -j
ctest --test-dir build/mujoco_sim_sensor --output-on-failure
```
