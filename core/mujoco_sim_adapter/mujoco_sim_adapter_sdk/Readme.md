# mujoco_sim_adapter_sdk

`mujoco_sim_adapter_sdk` 是 V1 同步控制器 Adapter 的 C++ SDK。它封装了
eCAL registry service、Tick 订阅、Heartbeat/Commit 发布和重复 Tick 的幂等处理，
调用方只需提供“一个 Tick 生成一组 actuator command”的回调。

**本包只安装 C++ 库和头文件，不生成、安装任何独立可执行程序。** 可直接运行的参考
程序由 [mujoco_sim_sample_adapter](../mujoco_sim_sample_adapter/Readme.md) 提供。

## 职责与边界

SDK 当前负责：

- 使用 `server_name` 定位 registry service，并发送 `RegisterController` 和
  `ActivateController` 请求。
- 注册调用方声明的 observation sensor 和 command actuator，保存服务端返回的 session、
  epoch 与资源布局。
- 创建 controller 专属 Tick 订阅，以及共享的 Heartbeat、Commit 发布端。
- 在 `Run()` 安装控制回调后激活 controller；对每个新 Tick 先发布 Heartbeat，再发布
  包含整组 command 的 Commit。
- 校验 Tick 的 server/controller/session/epoch/protocol/layout 与注册结果一致；串行化 Tick
  handler，并只对 identity 与 payload 一致的重传 Tick 重发缓存 Commit。
- 在进程尚未初始化 eCAL 时代为初始化，并仅在自己拥有该初始化时于析构阶段调用
  `eCAL::Finalize()`。
- `Run()` 返回和析构前停止 Tick 接收并等待所有 in-flight callback 完成，确保 Heartbeat/
  Commit 发布结束后再销毁 eCAL 对象。

边界如下：

- SDK 不加载 MuJoCo 模型、不持有 `mjModel`/`mjData`、不推进仿真，也不提供 GUI。
  物理状态和 Tick/Commit Barrier 均由 server 管理。
- SDK 不从服务端自动选择资源；`observation_names` 和 `command_names` 必须由调用方给出。
- SDK 不实现控制算法。Tick handler 在 eCAL 接收回调线程中同步执行，多个接收回调之间
  会串行化，且 handler 应在 controller timeout 内返回。
- SDK 仅把 handler 返回值编码为 Commit。除“结果为空”外，它不检查 command 是否完整、
  是否属于当前 controller、是否重复、是否为有限值或是否落在 actuator control range；
  这些约束由 server 校验。
- 当前 `Stop()` 不发送 `UnregisterController`；它停止接受新 Tick 并唤醒 `Run()`，随后
  `Run()` 或析构路径排空 callback 并销毁本地 eCAL 通道。
- 本包不包含命令行解析、配置文件解析、进程信号处理或自动重连逻辑。

协议消息和命名规则见
[mujoco_sim_protocol](../../mujoco_sim_protocol/Readme.md)。

## 依赖与构建

生产代码要求 CMake 3.22、C++20、eCAL 和 `mujoco_sim_protocol`；协议包继续提供
Protobuf 依赖。启用测试时还需要 GTest。SDK 本身不依赖 MuJoCo、Core 或 Server。

以下命令均从仓库根目录执行：

```bash
colcon build \
  --packages-up-to mujoco_sim_adapter_sdk \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

只构建生产库时可将 `BUILD_TESTING` 设为 `OFF`。更完整的工作区依赖说明见
[仓库 README](../../../Readme.md)。

## 公开 Target 与 API

安装后使用 CMake config package：

```cmake
find_package(mujoco_sim_adapter_sdk CONFIG REQUIRED)
target_link_libraries(my_adapter PRIVATE mujoco_sim::adapter_sdk)
```

公开 target 为 `mujoco_sim::adapter_sdk`，公开头文件为
[`<mujoco_sim/adapter_sdk/sync_adapter_client.hpp>`](include/mujoco_sim/adapter_sdk/sync_adapter_client.hpp)。
该 target 公开传递 `mujoco_sim::protocol` 和 `eCAL::core`。

主要 API：

| API | 当前行为 |
|---|---|
| `AdapterOptions` | 描述 server/controller、申请的资源和两个 timeout。 |
| `CommandValue` | 一个标量 actuator command，由 `name` 和 `double value` 组成。 |
| `TickHandler` | `std::function<std::vector<CommandValue>(const Tick&)>`。 |
| `BuildCommit()` | 复制 Tick header/controller，生成 `COMMIT_OK`，每个 `CommandValue` 编码为一个单值 command block。 |
| `BuildHeartbeat()` | 复制 Tick header/controller 生成 Heartbeat。 |
| `SyncAdapterClient::Start()` | 校验必要选项、初始化通信、注册 controller 并创建 pub/sub。 |
| `SyncAdapterClient::Run()` | 安装 handler、激活 controller 并阻塞处理 Tick。 |
| `SyncAdapterClient::Stop()` | 设置终止标志并唤醒 `Run()`。 |
| `SyncAdapterClient::registration()` | 返回最近一次成功注册得到的 `RegisterControllerResponse` 常量引用。 |

`AdapterOptions` 默认值与约束：

| 字段 | 默认值 | 说明 |
|---|---:|---|
| `server_name` | `"default"` | 必须与 server namespace 一致。 |
| `controller_id` | 空 | 必填；空值会使 `Start()` 失败。 |
| `observation_names` | 空列表 | 允许不申请 observation。 |
| `command_names` | 空列表 | 至少一个；空列表会使 `Start()` 失败。 |
| `timeout_ms` | `1000` | 注册给 server 的每 Tick Commit 等待上限；传 `0` 时 server 使用自身默认值。 |
| `service_timeout_ms` | `5000` | 每次 registry service 调用的总等待上限。 |

`SyncAdapterClient` 不可复制。`Start()` 的 `error_message` 必须指向有效字符串；成功后可通过
`registration()` 检查服务端确认的 session、epoch 和 `layout`。

## Start / Run / Stop 生命周期

1. 构造 `AdapterOptions` 和 `SyncAdapterClient`。
2. 调用 `Start(&error)`。它先检查 `controller_id` 和 `command_names`，必要时初始化 eCAL，
   等待 registry service，在 `service_timeout_ms` 内注册 controller，然后创建 Commit、
   Heartbeat publisher 和 Tick subscriber。此时 controller 仍为 `PROVISIONAL`，尚未激活。
3. 调用 `Run(handler, max_ticks)`。它先保存 handler，等待 100 ms 让 pub/sub 完成发现，再
   调用 `ActivateController`。激活成功后阻塞，直到达到最大 Tick 数、调用 `Stop()`，或
   eCAL 不再处于可运行状态。返回前会停止 subscriber，并等待已经进入的 Tick callback
   完成 Commit 发布。
4. `Stop()` 可从其他线程调用。它只请求本地 `Run()` 结束；析构函数也会调用一次
   `Stop()`。

`max_ticks == 0` 表示不按 Tick 数自动退出；大于零时只统计首次处理的新 Tick。`Run()`
在达到 `max_ticks` 或收到 `Stop()` 后返回 `0`，激活失败或 eCAL 非正常结束时返回 `1`。
激活失败的详细字符串当前不会由 `Run()` 对外返回。

当前对象按单次生命周期实现：成功 `Start()` 后调用一次 `Run()`，结束后析构。`Stop()`
标志、已处理计数和 Commit 缓存不会重置，因此不要用同一对象再次 `Start()` 或 `Run()`。

## Tick 与重复 Tick

收到 Tick 后先校验 `controller_id`、server name、session、epoch、协议版本、ACTIVE 状态和
完整 resource layout 均与注册结果一致。校验通过后按 Tick stream 与 ID 处理：

- 首次收到可处理的新 Tick：发布 Heartbeat，调用 handler，构造并缓存 Tick/Commit，已处理
  计数加一，然后发布 Commit。handler 执行期间其他 Tick 回调会等待，不能并发进入 handler。
- 再次收到相同 server/session/epoch/protocol/controller/tick identity，且除允许变化的
  timestamp 外 layout/observation payload 相同：若计算尚未结束则等待；结束后发布
  Heartbeat 并原样重发缓存 Commit，不调用 handler，也不增加计数。
- identity 相同但 payload 改变，或同一 stream 中收到小于缓存值的旧 Tick：直接忽略。
- Tick 无法反序列化、超出消息大小、注册身份/layout 不匹配或 handler 尚未安装：直接忽略。

handler 返回空列表时，SDK 仍会发送 Commit，但状态为 `COMMIT_ERROR`，错误信息为
`adapter handler returned no commands`。服务端会将该结果视为 controller 错误。因此
handler 应为 Tick layout 中每个已认领 actuator 返回且只返回一个有限标量值。

重复 Tick 机制保证并发和串行重传都使用相同 command payload，不会让有状态控制算法对
同一 Tick 推进两次。由于 handler 整体串行执行，不同 Tick 也不会并发修改 handler 状态；
等待期间若更大的 Tick 已完成，随后到达的较小 Tick 会按旧 Tick 规则忽略。

## 完整最小示例

以下两个文件构成一个完整的固定值 Adapter。假设将它们放在仓库根目录下的
`adapter_example/`。

`adapter_example/main.cpp`：

```cpp
#include <iostream>
#include <string>
#include <vector>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

int main() {
  mujoco_sim::adapter_sdk::AdapterOptions options;
  options.server_name = "default";
  options.controller_id = "minimal_controller";
  options.observation_names = {"hinge_pos", "hinge_vel"};
  options.command_names = {"hinge_motor"};

  mujoco_sim::adapter_sdk::SyncAdapterClient client(options);
  std::string error;
  if (!client.Start(&error)) {
    std::cerr << "adapter start failed: " << error << '\n';
    return 1;
  }

  const int result = client.Run(
      [](const mujoco_sim::v1::Tick& tick) {
        std::vector<mujoco_sim::adapter_sdk::CommandValue> commands;
        for (const auto& descriptor : tick.layout().commands()) {
          commands.push_back({descriptor.name(), 0.1});
        }
        return commands;
      },
      5);
  client.Stop();
  return result;
}
```

`adapter_example/CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.22)
project(minimal_mujoco_adapter LANGUAGES CXX)

find_package(mujoco_sim_adapter_sdk CONFIG REQUIRED)

add_executable(minimal_mujoco_adapter main.cpp)
target_compile_features(minimal_mujoco_adapter PRIVATE cxx_std_20)
target_link_libraries(minimal_mujoco_adapter
  PRIVATE mujoco_sim::adapter_sdk
)
```

先构建并加载 SDK，再构建示例：

```bash
source install/setup.bash
cmake -S adapter_example -B adapter_example/build
cmake --build adapter_example/build -j
```

## 联调顺序

SDK 没有自身可执行程序，联调时运行上面的示例程序或
[sample adapter](../mujoco_sim_sample_adapter/Readme.md)。先准备 server：

```bash
colcon build --packages-up-to mujoco_sim_server
source install/setup.bash
```

两个终端都保持在仓库根目录，并各自加载环境。先启动终端 1 的 server：

```bash
source install/setup.bash
mujoco-sim-server \
  --headless \
  --server-name default \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 5 \
  --startup-timeout-ms 5000
```

然后在 server 的 5 秒 startup timeout 内启动终端 2 的 Adapter：

```bash
source install/setup.bash
./adapter_example/build/minimal_mujoco_adapter
```

联调时必须保证 `server_name` 一致，controller ID 未被占用，资源名称存在且 actuator
尚未被其他 controller 认领。server 一旦打开第一个 Tick，就不再接受运行时注册或激活。
示例两端都配置 5 个 Tick，因此正常完成后都会自动返回。

## 测试

SDK 当前注册一个 CTest：`adapter_sdk_test`。它验证 Commit/Heartbeat 编码、并发重复 Tick
单次计算、timestamp 可变重传、冲突 payload/旧 Tick 拒绝、跨 epoch 缓存隔离、注册身份和
layout 校验，以及停止接收时等待 in-flight handler 完成。

```bash
colcon test --packages-select mujoco_sim_adapter_sdk
colcon test-result --verbose
```

该单元测试不启动 eCAL server。真实注册、激活和 Tick/Commit 链路由
[sample adapter 的端到端测试](../mujoco_sim_sample_adapter/Readme.md#测试) 覆盖。
