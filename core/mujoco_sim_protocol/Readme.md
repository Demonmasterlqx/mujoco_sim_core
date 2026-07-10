# mujoco_sim_protocol

定义 V1 eCAL 通信使用的 Protobuf 协议，并生成 C++ 绑定。

职责：

- `.proto` 是协议真相。
- 提供 Register/Activate/Unregister、Tick、Commit、Heartbeat、Status 消息。
- 提供 registry service 和 runtime topic 命名辅助函数。

本包不依赖 MuJoCo、Core、Server 或任何 adapter。
