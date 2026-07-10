# mujoco_sim_sample_adapter

一个简单的 adapter 样例，展示如何使用 [mujoco_sim_adapter_sdk](../mujoco_sim_adapter_sdk/Readme.md)

默认资源：

- observations: `hinge_pos,hinge_vel`
- command: `hinge_motor`

运行：

```bash
source install/setup.bash
mujoco-sim-sample-adapter --ticks 5 --command-value 0.1
```
