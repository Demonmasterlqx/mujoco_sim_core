# mujoco_sim_protocol

`mujoco_sim_protocol` 是 MuJoCo 仿真器 V1 lockstep 接口的协议包。它以
[sync.proto](proto/mujoco_sim/v1/sync.proto) 为协议真相，生成 C++ Protobuf
绑定，并提供 registry service 与运行期 topic 的统一命名函数。

| 项目 | 值 |
| --- | --- |
| 包版本 | `0.1.0` |
| colcon 构建类型 | `cmake` |
| C++ 标准 | C++20 |
| 协议版本 | `1.2`（向 1.1 Controller 兼容） |
| 许可证 | `Apache-2.0` |

## 职责与边界

本包负责：

- 定义 legacy Controller 与通用 Adapter/Observer 注册、激活、注销消息。
- 定义 `Tick`、`ObservationFrame`、原子 `AdapterCommit`、两类 Heartbeat 和统一状态消息。
- 定义静态 sensor stream catalog、相机内参/分割标签、frame convention 与 stream status。
- 提供小 Protobuf header + raw payload 的二进制 framing，避免把大图像/点云编码为 repeated
  Protobuf scalar。
- 定义错误码、资源类型、控制器状态、服务端状态与提交状态。
- 生成并安装 C++ Protobuf 绑定。
- 提供协议版本常量、registry 方法名以及 service/topic 命名函数。

本包不负责：

- 不连接 eCAL，也不创建 publisher、subscriber 或 RPC server/client。
- `.proto` 中没有 Protobuf `service` 声明；registry 的调用方式由传输层实现。
- 不实现注册状态机、actuator 所有权、Tick/Commit barrier 或超时策略。
- 不加载 MuJoCo 模型，也不依赖 MuJoCo、Core、Server 或 Adapter SDK。
- 不提供配置解析、线程、事件循环或可执行程序。

当前仓库使用 eCAL 承载这些消息，但 Protobuf 消息和命名辅助函数本身不依赖
eCAL。

## 依赖与构建

构建配置以 [CMakeLists.txt](CMakeLists.txt) 和 [package.xml](package.xml) 为准。

| 依赖 | 用途 |
| --- | --- |
| CMake `>= 3.22` | 配置、生成与安装 |
| 支持 C++20 的编译器 | 编译库和消费者代码 |
| Protobuf | `protoc` 代码生成及 `protobuf::libprotobuf` 运行库 |
| GTest | 仅在 `BUILD_TESTING=ON` 时构建测试 |
| colcon | 仓库工作区的构建入口，不是运行时依赖 |

以下命令均从仓库根目录运行：

```bash
colcon build \
  --packages-select mujoco_sim_protocol \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

安装内容包括库、公开头文件、生成的 `sync.pb.h`、原始 `.proto` 文件，以及
供 `find_package` 使用的 CMake package config。

## 是否可独立启动

不能启动。本包只生成一个库，不安装可执行程序或 launch 文件。

它可以被单独构建、测试和链接。要启动完整仿真进程，应使用
[mujoco_sim_server](../mujoco_sim_server/Readme.md)；控制器侧通常通过
[mujoco_sim_adapter_sdk](../mujoco_sim_adapter/mujoco_sim_adapter_sdk/Readme.md)
接入。

## 公开 CMake target、头文件与 API

### CMake target

安装后的消费者只需链接导出 target `mujoco_sim::protocol`：

```cmake
cmake_minimum_required(VERSION 3.22)
project(protocol_consumer LANGUAGES CXX)

find_package(mujoco_sim_protocol CONFIG REQUIRED)

add_executable(protocol_consumer main.cpp)
target_link_libraries(protocol_consumer PRIVATE mujoco_sim::protocol)
```

该 target 公开 C++20、协议头文件目录和 `protobuf::libprotobuf` 依赖。

### 公开头文件

| 头文件 | 内容 |
| --- | --- |
| `<mujoco_sim/protocol/topics.hpp>` | 版本常量、service/topic 命名函数、registry 方法名 |
| `<mujoco_sim/protocol/stream_frame.hpp>` | raw stream framing 序列化/解析、24-byte 前缀、1 MiB header 与 512 MiB payload 上限 |
| `<mujoco_sim/v1/sync.pb.h>` | 由 `sync.proto` 生成的全部 C++ 消息与枚举 |

生成类型位于 `mujoco_sim::v1` 命名空间，命名辅助 API 位于
`mujoco_sim::protocol` 命名空间。

### 消息与枚举

| 类型 | 当前用途 |
| --- | --- |
| `ErrorCode` | `OK`、参数/资源/版本/session/epoch/tick/command 错误、超时和内部错误等结果 |
| `ResourceType` | 区分 observation sensor 与 command actuator |
| `AdapterRole` | 区分参与 Barrier 的 `CONTROLLER` 与非 Barrier `OBSERVER` |
| `StreamFormat`、`FrameConvention`、`StreamState` | 大数据 raw payload 的格式、坐标约定和运行状态 |
| `ControllerState` | `PROVISIONAL`、`ACTIVE`、`ERROR`、`UNREGISTERED`、`TIMED_OUT` |
| `ServerState` | `IDLE`、`RUNNING`、`ERROR_PAUSED`、`STOPPED` |
| `CommitStatus` | `COMMIT_OK` 或 `COMMIT_ERROR` |
| `Header` | `server_name`、`session_id`、`epoch`、`tick_id`、wall 时间戳、协议版本和可选仿真时间 |
| `ResourceDescriptor` | 资源名、类型、维度、MuJoCo stage/interval/delay/object/unit/interpolation 元数据和可选 control range |
| `ResourceLayout` | 某控制器获配的 observations 与 commands |
| `GetAvailableResourcesRequest/Response` | 查询服务端可用资源与协商后的协议版本 |
| `RegisterControllerRequest/Response` | 申请资源并取得 `session_id`、`epoch` 与 layout |
| `ActivateControllerRequest/Response` | 将已注册控制器从 provisional 转为 active |
| `UnregisterControllerRequest/Response` | 注销控制器 |
| `Register/Activate/UnregisterAdapter*` | role-aware Controller/Observer 生命周期；Controller 仍可使用 legacy API |
| `ObservationDelivery` | Observer 的 every-N-step 或 simulation-period 二选一规则 |
| `ObservationBlock` | 一个命名 observation 的 `double` 数组 |
| `ActuatorCommandBlock` | 一个命名 actuator 的 `double` 数组 |
| `Tick` | header、controller、layout、observations 与 controller state |
| `ObservationFrame` | post-step Observer 小数据、独立 sequence，不要求 Commit |
| `AdapterCommit` | header、controller、提交状态、整组 actuator commands 与错误文本 |
| `Heartbeat` | controller 及其当前协议 header |
| `AdapterHeartbeat` | role-aware keepalive 与 Observer 最后接收 sequence |
| `ControllerStatus` | session、最后 heartbeat/Commit Tick、状态起始 Tick、累计超时数、Barrier 成员标志、状态与说明 |
| `AdapterStatus` | Controller/Observer 的统一 lifecycle 视图 |
| `StreamDescriptor`、`GetSensorStreams*` | 静态 stream topic、shape、stride、frame、rate、内参与 label catalog |
| `StreamFrameHeader`、`StreamStatus` | 每帧 identity/sequence/capture/drop 元数据和运行指标 |
| `ServerStatus` | 服务端状态、legacy controllers、统一 adapters 与 streams |

完整字段号、枚举值和 wire contract 以 [sync.proto](proto/mujoco_sim/v1/sync.proto)
为准。

### 命名 API

版本常量为 `kProtocolMajor == 1`、`kProtocolMinor == 2`。命名函数不会访问
传输层：

| API | `server_name=default`、`controller_id=arm` 时的结果 |
| --- | --- |
| `RegistryServiceName(server_name)` | `mujoco_sim.default.registry` |
| `TickTopicName(server_name, controller_id)` | `mujoco_sim/default/tick/arm` |
| `CommitTopicName(server_name)` | `mujoco_sim/default/commit` |
| `HeartbeatTopicName(server_name)` | `mujoco_sim/default/heartbeat` |
| `AdapterHeartbeatTopicName(server_name)` | `mujoco_sim/default/adapter_heartbeat` |
| `ObservationTopicName(server_name, adapter_id)` | `mujoco_sim/default/observation/arm` |
| `SensorStreamTopicName(server_name, stream_name)` | `mujoco_sim/default/sensor/arm` |
| `StatusTopicName(server_name)` | `mujoco_sim/default/status` |

除 legacy Controller 方法外，registry 还定义 `RegisterAdapter`、`ActivateAdapter`、
`UnregisterAdapter` 和 `GetSensorStreams` 方法名常量。

## 最小可用示例

下面的消费者同时演示 topic 命名、构造 `AdapterCommit`、Protobuf 二进制序列化
与反序列化。其 CMake 配置使用上一节的 `mujoco_sim::protocol` target。

```cpp
#include <iostream>
#include <string>

#include <mujoco_sim/protocol/topics.hpp>
#include <mujoco_sim/v1/sync.pb.h>

int main() {
  const std::string server_name = "default";
  const std::string controller_id = "arm";

  std::cout << mujoco_sim::protocol::RegistryServiceName(server_name) << '\n';
  std::cout << mujoco_sim::protocol::TickTopicName(server_name, controller_id)
            << '\n';
  std::cout << mujoco_sim::protocol::CommitTopicName(server_name) << '\n';

  mujoco_sim::v1::AdapterCommit outgoing;
  auto* header = outgoing.mutable_header();
  header->set_server_name(server_name);
  header->set_session_id(42);
  header->set_epoch(7);
  header->set_tick_id(3);
  header->set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  header->set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  outgoing.set_controller_id(controller_id);
  outgoing.set_status(mujoco_sim::v1::COMMIT_OK);

  auto* command = outgoing.add_commands();
  command->set_name("hinge_motor");
  command->add_values(0.25);

  std::string payload;
  if (!outgoing.SerializeToString(&payload)) {
    std::cerr << "serialize failed\n";
    return 1;
  }

  mujoco_sim::v1::AdapterCommit incoming;
  if (!incoming.ParseFromString(payload)) {
    std::cerr << "parse failed\n";
    return 2;
  }

  std::cout << "tick=" << incoming.header().tick_id()
            << ", actuator=" << incoming.commands(0).name()
            << ", value=" << incoming.commands(0).values(0) << '\n';
  return 0;
}
```

`payload` 只是 Protobuf wire bytes，不包含 topic、消息类型或额外帧边界；具体传输层
必须自行保留消息边界，并确保发布端和订阅端约定相同的消息类型。

### Raw stream framing

图像和点云使用 `SerializeStreamFrame()`/`ParseStreamFrame()`。一条 transport message 由
以下内容连续组成：

1. 8-byte ASCII magic `MJSTREAM`。
2. little-endian framing major/minor、32-bit header 长度和 64-bit payload 长度，共补足为
   固定 24-byte 前缀。
3. 序列化后的 `StreamFrameHeader` Protobuf。
4. 未压缩 raw payload。

解析结果中的 payload 是对输入 buffer 的 borrowed `std::span<const std::byte>`；输入消息
失效前必须完成使用或显式复制。当前 framing 版本固定为 `1.0`，解析器严格拒绝其他
major/minor；序列化后的 Protobuf header 上限为 1 MiB，raw payload 上限为 512 MiB，
序列化和解析两条路径执行相同限制。RGB 为
top-left `RGB8`；`DEPTH32F` 和 `XYZ32F` 是 little-endian IEEE-754 binary32，
`SEGMENTATION32U` 是 little-endian uint32。depth 表示米制 optical Z，点云为 packed XYZ；
具体 shape/stride/frame convention 必须与 catalog descriptor 完全一致。

## 关键约束

- Proto3 枚举的默认值是各枚举的 `*_UNSPECIFIED == 0`；尤其
  `ErrorCode::OK == 1`，不能把未设置的 `code` 当成成功。
- 当前协议常量是 `1.2`。协议包只声明版本；版本校验和 minor version 协商由 Core
  或其他消费者实现。
- V1.1 在保持既有枚举数值和字段号不变的基础上新增 `ControllerState::TIMED_OUT == 5`；
  `ControllerStatus` 新增 optional `last_commit_tick_id`、`state_since_tick_id`、
  `timeout_count` 和 `required_for_barrier`。`Tick::controller_state` 用于区分正常控制 Tick 与
  超时恢复 Tick。
- Core 与协议 1.0 controller 协商到 minor 0 时，超时恢复 Tick 的线上 state 降级为旧 SDK
  可识别的 `ACTIVE`；这是宿主的兼容策略，不改变 V1.1 枚举定义。
- V1.2 只追加消息、枚举和新字段；legacy Controller registry、Tick、Commit 与 Heartbeat
  的既有字段号和行为保持不变。`Header.simulation_time_s` 是 optional，旧端会忽略它。
- Controller Tick 中的 observation 是同步控制契约的一部分；`ObservationFrame` 是新增的
  sensor-only 小数据通道，不能替代或削弱 Controller Tick 的 observation。
- `Header` 的 `server_name`、`session_id`、`epoch` 和 `tick_id` 共同标识一次
  lockstep 交互。仅成功解析 Protobuf 不代表这些字段在业务上有效。
- `AdapterCommit` 表达一组命令，但消息类本身不保证原子性、资源所有权或 barrier；
  这些语义由 [mujoco_sim_core](../mujoco_sim_core/Readme.md) 校验。
- 命名函数直接拼接输入，不检查空字符串，也不转义 `.`、`/` 等分隔符。调用方必须保证
  `server_name` 和 `controller_id` 在整个系统中一致且适合当前传输层。
- 生成头文件位于构建目录并在安装时复制，不作为手写源码维护。协议变更应修改
  `sync.proto`，且兼容演进时不能复用既有字段号或枚举数值。

## 测试

从仓库根目录运行：

```bash
colcon build \
  --packages-select mujoco_sim_protocol \
  --cmake-args -DBUILD_TESTING=ON
colcon test \
  --packages-select mujoco_sim_protocol \
  --event-handlers console_direct+
colcon test-result \
  --test-result-base build/mujoco_sim_protocol \
  --verbose
```

[protocol_test.cpp](test/protocol_test.cpp) 当前覆盖 legacy 与新增 service/topic 命名、
Controller/Observer/Stream 消息序列化、仿真时间、V1.1 的 18 组固定 golden wire bytes、
85 个旧字段号和 31 个旧枚举值，以及 raw frame 固定 golden bytes、round-trip、坏
Protobuf header、major/minor、前缀/header 长度冲突、magic 和 payload 上限拒绝。

## 相关包

- [仓库总览](../../Readme.md)
- [mujoco_sim_core](../mujoco_sim_core/Readme.md)：协议状态机、资源所有权与 lockstep barrier
- [mujoco_sim_server](../mujoco_sim_server/Readme.md)：当前 eCAL 服务端与进程入口
- [mujoco_sim_adapter_sdk](../mujoco_sim_adapter/mujoco_sim_adapter_sdk/Readme.md)：控制器侧 eCAL SDK
- [mujoco_sim_sample_adapter](../mujoco_sim_adapter/mujoco_sim_sample_adapter/Readme.md)：SDK 使用样例
