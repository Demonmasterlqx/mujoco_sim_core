# mujoco_sim_adapter_sdk

C++ 同步控制 Adapter SDK。

职责：

- 等待并调用 server registry service。
- 注册 observation sensors 和 command actuators。
- 在安装 Tick handler 后激活 controller。
- 订阅 per-controller Tick。
- 发布 Heartbeat 和原子 Commit。
- 对重复 Tick 重发上一份 Commit，支持 server 侧幂等重传。

用户只需要实现 `Tick -> commands` 控制回调。
