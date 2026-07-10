# mujoco_sim_core

`mujoco_sim_core` 是与传输层无关的 MuJoCo V1 lockstep 仿真核心。它持有真实
`mjModel`/`mjData`，从模型建立资源目录，并在所有 active controller 完成当前
Tick 的 commit barrier 后执行一次 `mj_step()`。

| 项目 | 值 |
| --- | --- |
| 包版本 | `0.1.0` |
| colcon 构建类型 | `cmake` |
| C++ 标准 | C++20 |
| MuJoCo | 随包提供的 Linux x86-64 MuJoCo `3.10.0` |
| 许可证 | `Apache-2.0` |

## 职责与边界

本包负责：

- 通过 `mj_loadXML()` 加载 MJCF XML，可从具名 keyframe 初始化，并创建、重置和持有
  唯一真实仿真状态。
- 从具名 MuJoCo sensor 和 actuator 建立 observation/command 资源目录。
- 校验资源发现、控制器注册、激活、注销、session、epoch 与 tick。
- 对 command actuator 实施单控制器独占所有权。
- 生成每个 active controller 的 `Tick`，收集完整 `AdapterCommit`，完成 barrier
  后写入 `data->ctrl` 并调用一次 `mj_step()`。
- 检查 open tick 的 commit 超时、记录 heartbeat tick，并生成 `ServerStatus`。
- 为可视化调用方创建与真实仿真状态隔离的 MuJoCo 快照。

本包不负责：

- 不实现 eCAL 或其他 IPC/RPC，不创建 publisher、subscriber 或 registry service。
- 不提供线程、事件循环、自动 Tick 循环、重试调度或进程生命周期。
- 不解析命令行/YAML，不提供 GUI，也不实现控制器算法。
- 不安装可执行程序；仓库中的进程入口属于
  [mujoco_sim_server](../mujoco_sim_server/Readme.md)。
- 快照不是持久化、网络同步或仿真回滚接口。

## 依赖与构建

构建配置以 [CMakeLists.txt](CMakeLists.txt) 和 [package.xml](package.xml) 为准。

| 依赖 | 用途 |
| --- | --- |
| CMake `>= 3.22` | 配置、生成与安装 |
| 支持 C++20 的编译器 | 编译库和消费者代码 |
| [mujoco_sim_protocol](../mujoco_sim_protocol/Readme.md) | V1 消息、错误码和版本常量 |
| MuJoCo `3.10.0` | 模型加载、状态复制和 `mj_step()`；默认使用包内二进制分发 |
| GTest | 仅在 `BUILD_TESTING=ON` 时构建测试 |
| colcon | 仓库工作区的构建入口，不是运行时依赖 |

以下命令均从仓库根目录运行；`--packages-up-to` 会先构建协议包：

```bash
colcon build \
  --packages-up-to mujoco_sim_core \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

默认 `MUJOCO_ROOT` 是
`core/mujoco_sim_core/mujoco/linux/x86/mujoco-3.10.0`。当前 CMake 明确查找
`lib/libmujoco.so.3.10.0`，因此默认分发仅适用于兼容的 Linux x86-64 环境。可用
`-DMUJOCO_ROOT=/absolute/path` 覆盖目录，但替代目录仍必须提供相同文件名和兼容的
头文件布局。

安装步骤会安装 Core 头文件、MuJoCo 的 `mujoco/` 头文件、
`libmujoco.so.3.10.0` 以及 CMake package config。未设置 `BUILD_SHARED_LIBS` 时 Core
默认构建为静态库；显式启用该选项时 `add_library()` 会生成共享库。安装后的 package
config 会创建指向同一安装前缀中 MuJoCo 动态库的 `mujoco::mujoco` imported target。

## 是否可独立启动

不能启动。本包只导出库，没有 `main()`、可执行程序或 launch 文件。

它可以在不依赖 eCAL 的进程内程序中直接使用；但调用方必须自行驱动注册、
`OpenTick()`、commit 收集、超时检查和下一 Tick。需要现成的 eCAL 进程入口、配置和
GUI 时使用 [mujoco_sim_server](../mujoco_sim_server/Readme.md)。

## 公开 CMake target、头文件与 API

### CMake target

安装后的消费者链接导出 target `mujoco_sim::core`：

```cmake
cmake_minimum_required(VERSION 3.22)
project(core_consumer LANGUAGES CXX)

find_package(mujoco_sim_core CONFIG REQUIRED)

add_executable(core_consumer main.cpp)
target_link_libraries(core_consumer PRIVATE mujoco_sim::core)
```

package config 会查找 `mujoco_sim_protocol`，并为安装的 MuJoCo 动态库建立
`mujoco::mujoco` imported target。消费者应链接 `mujoco_sim::core`，无需硬编码
MuJoCo 库路径。

### 公开头文件

| 头文件 | 内容 |
| --- | --- |
| `<mujoco_sim/core/simulation_core.hpp>` | Core 的全部手写公开 C++ API |
| `<mujoco_sim/v1/sync.pb.h>` | 由协议 target 传递的 V1 Protobuf 类型 |
| `<mujoco/mujoco.h>` | 访问快照返回的 `mjModel*`/`mjData*` 具体字段时需要 |

### 公开类型与方法

API 位于 `mujoco_sim::core` 命名空间。

| 类型/API | 行为 |
| --- | --- |
| `CoreOptions` | `server_name` 默认 `default`；`default_timeout_ms` 默认 `1000` |
| `ModelLoadOptions` | 可选的 `initial_keyframe`；空字符串保持默认 reset 行为 |
| `CommitResult` | 返回 `code`、说明文本、是否为重复提交及本次是否实际 step |
| `SimulationSnapshot` | move-only 的隔离 MuJoCo model/data 副本，提供 `model()`、`data()` |
| `SimulationCore(options)` | 构造 move-only Core；不自动加载模型 |
| `LoadModel()`、`IsModelLoaded()` | 加载 MJCF XML，可选初始 keyframe，并查询模型状态 |
| `observations()`、`commands()` | 读取当前模型建立的资源目录 |
| `GetAvailableResources()` | 校验 server/协议版本并返回资源目录 |
| `RegisterController()` | 校验并分配资源，返回 session、epoch、layout 和 provisional 状态 |
| `ActivateController()` | 在首个 Tick 前将 provisional controller 设为 active |
| `UnregisterController()` | 在 Tick 关闭时释放控制器及 actuator 所有权 |
| `OpenTick()` | 为所有 active controller 生成当前 Tick，并开启 barrier/超时计时 |
| `SubmitCommit()` | 校验并暂存 commit；最后一个 active controller 到达时执行一次 step |
| `AcceptHeartbeat()` | 对 header 有效的已知控制器记录当前 tick，不返回错误 |
| `CheckTickTimeout(now)` | 检查 open tick 中尚未提交的控制器是否超时 |
| `MakeStatus()` | 生成当前服务端和控制器状态消息 |
| `CreateSnapshot()`、`UpdateSnapshot()` | 创建和刷新隔离的可视化副本 |
| 状态 getter | `epoch()`、`current_tick_id()`、`simulation_time()`、`server_state()`、`tick_open()`、`active_controller_count()` |

`LoadModel()` 保留原有 `(model_path, error_message)` 签名，并新增
`(model_path, ModelLoadOptions, error_message)` 重载。所有 `LoadModel()`、
`CreateSnapshot()` 和 `UpdateSnapshot()` 的 `error_message` 均可传 `nullptr`；需要诊断
信息时传入 `std::string*`。

### 初始 keyframe

`initial_keyframe` 为空时，旧重载和新重载都执行原有的 `mj_resetData()` 与
`mj_forward()` 流程。指定名称时，Core 在临时 model/data 上用 `mjOBJ_KEY` 查找 keyframe，
调用 `mj_resetDataKeyframe()`，将整个 `data->ctrl` 清零，再调用 `mj_forward()`：

```cpp
mujoco_sim::core::ModelLoadOptions load_options;
load_options.initial_keyframe = "ready";

std::string error;
if (!simulation.LoadModel(model_path, load_options, &error)) {
  // error 包含缺失的 keyframe 名称或 MuJoCo 加载诊断。
}
```

keyframe 的 time、qpos、qvel、act 等状态会成为初始状态；其中保存的 ctrl 不会成为启动
命令。只有临时状态完成初始化后才替换当前模型。XML 加载、data 分配或 keyframe 查找失败
时，现有 model/data、资源目录、controller、epoch、Tick、快照 generation 和服务状态均不变。

## 最小可用示例

以下程序使用仓库测试模型演示模型加载和一个 controller 的完整 lockstep 生命周期。
模型路径以仓库根目录为当前工作目录；消费者的 CMake 配置使用上一节的
`mujoco_sim::core` target。

```cpp
#include <iostream>
#include <string>

#include <mujoco_sim/core/simulation_core.hpp>
#include <mujoco_sim/protocol/topics.hpp>

int main() {
  mujoco_sim::core::CoreOptions options;
  options.server_name = "default";
  options.default_timeout_ms = 1000;
  mujoco_sim::core::SimulationCore simulation(options);

  std::string error;
  if (!simulation.LoadModel(
          "core/mujoco_sim_core/test/testdata/simple_actuator.xml", &error)) {
    std::cerr << "load failed: " << error << '\n';
    return 1;
  }
  std::cout << "observations=" << simulation.observations().size()
            << ", commands=" << simulation.commands().size() << '\n';

  mujoco_sim::v1::RegisterControllerRequest register_request;
  register_request.set_server_name("default");
  register_request.set_controller_id("arm");
  register_request.set_protocol_major(mujoco_sim::protocol::kProtocolMajor);
  register_request.set_protocol_minor(mujoco_sim::protocol::kProtocolMinor);
  register_request.add_observation_names("hinge_pos");
  register_request.add_observation_names("hinge_vel");
  register_request.add_command_names("hinge_motor");

  const auto registration =
      simulation.RegisterController(register_request);
  if (registration.code() != mujoco_sim::v1::OK) {
    std::cerr << "register failed: " << registration.message() << '\n';
    return 2;
  }

  mujoco_sim::v1::ActivateControllerRequest activate_request;
  activate_request.set_controller_id(registration.controller_id());
  activate_request.set_session_id(registration.session_id());
  activate_request.set_epoch(registration.epoch());
  const auto activation = simulation.ActivateController(activate_request);
  if (activation.code() != mujoco_sim::v1::OK) {
    std::cerr << "activate failed: " << activation.message() << '\n';
    return 3;
  }

  const auto ticks = simulation.OpenTick();
  if (ticks.size() != 1) {
    std::cerr << "expected one tick\n";
    return 4;
  }

  mujoco_sim::v1::AdapterCommit commit;
  *commit.mutable_header() = ticks.front().header();
  commit.set_controller_id(registration.controller_id());
  commit.set_status(mujoco_sim::v1::COMMIT_OK);
  auto* command = commit.add_commands();
  command->set_name(registration.layout().commands(0).name());
  command->add_values(0.25);

  const auto result = simulation.SubmitCommit(commit);
  if (result.code != mujoco_sim::v1::OK || !result.stepped) {
    std::cerr << "commit failed: " << result.message << '\n';
    return 5;
  }
  std::cout << "tick=" << simulation.current_tick_id()
            << ", simulation_time=" << simulation.simulation_time() << '\n';

  mujoco_sim::v1::UnregisterControllerRequest unregister_request;
  unregister_request.set_controller_id(registration.controller_id());
  unregister_request.set_session_id(registration.session_id());
  unregister_request.set_epoch(registration.epoch());
  const auto unregistered =
      simulation.UnregisterController(unregister_request);
  return unregistered.code() == mujoco_sim::v1::OK ? 0 : 6;
}
```

多个 controller 时，每个 `SubmitCommit()` 都可能只返回“已接收”；只有最后一个
active controller 的有效 commit 使 `CommitResult::stepped` 为 `true`。调用方随后
显式调用下一次 `OpenTick()`。Core 不会在后台自动推进。

完整生命周期是：加载模型，发现/选择资源，注册为 `PROVISIONAL`，激活为 `ACTIVE`，
打开 Tick，收齐 commit barrier 并 step，然后打开下一 Tick 或在 Tick 关闭时注销。
Tick 打开期间，宿主应周期性调用 `CheckTickTimeout(std::chrono::steady_clock::now())`。

## 快照

`SimulationSnapshot` 当前专用于向可视化等只需副本的调用方提供隔离状态：

```cpp
#include <mujoco/mujoco.h>

std::string error;
auto snapshot = simulation.CreateSnapshot(&error);
if (!snapshot) {
  std::cerr << error << '\n';
  return 1;
}

std::cout << "snapshot time=" << snapshot->data()->time << '\n';

// Core 完成后续 step 后，由宿主显式刷新副本。
if (!simulation.UpdateSnapshot(snapshot.get(), &error)) {
  std::cerr << error << '\n';
  return 2;
}
```

- `CreateSnapshot()` 使用 `mj_copyModel()` 和 `mj_copyData()` 创建独立 model/data；
  修改快照不会修改 Core 的真实状态。
- 快照不会自动更新。`UpdateSnapshot()` 只把当前真实 `mjData` 复制到既有快照，
  不执行 step，也不把快照内容写回 Core。
- 快照只能由创建它的同一个 `SimulationCore`、同一模型 generation 刷新。Core 再次
  成功 `LoadModel()` 后，旧快照仍可自行析构或读取，但 `UpdateSnapshot()` 会拒绝它；
  另一个 Core 也不能刷新该快照。
- 快照是 move-only，`model()`/`data()` 返回的指针由快照持有，不能在快照销毁后使用。
- 该 API 没有序列化或 restore 操作，因此不能作为 checkpoint、持久化存档或回滚机制。
- Core 内部没有并发锁；若仿真线程可能同时 step，宿主必须在
  `CreateSnapshot()`/`UpdateSnapshot()` 与其他 Core 调用之间提供同步。

## 关键约束

- `LoadModel()` 只支持通过 `mj_loadXML()` 加载 XML，当前没有 MJB 加载 API。成功加载会按
  默认 reset 或指定 keyframe 初始化 data、清空 ctrl，随后清空 controllers、actuator owners
  和 pending commits，递增 epoch 和模型 generation，将 tick 归零并回到 `IDLE`；加载失败
  不会替换或修改当前模型状态。
- 资源目录只包含具名 sensor 和具名 actuator，未命名资源会被跳过。Observation 维度
  来自 `sensor_dim`；每个 command actuator 当前固定为一个 `double`，并按模型的
  `ctrlrange` 校验。
- controller 必须在首个 Tick 前注册和激活。注册至少要申请一个 command actuator；
  同一 actuator 不能同时属于两个 controller，资源名也不能在同一请求中重复。
- 首个 Tick 之后不支持新增注册或激活。注销只能在没有 open tick 时进行。
- commit 必须来自 active controller，且 server/session/epoch/tick 必须匹配；它必须将
  该 controller 获配的每个 actuator 恰好提交一次，每项恰好一个有限值，且不能超出
  control range。
- 所有 active controller 构成 barrier。收到最后一份有效 commit 前不会写
  `data->ctrl` 或 step；完成后统一写入命令并只调用一次 `mj_step()`。
- 同一 open tick 的同 payload 重复 commit 会被忽略；payload 不同会进入
  `ERROR_PAUSED`。身份字段匹配且 tick 已过期的提交按 stale duplicate 忽略。
- header 身份不匹配、open tick 中来自未知/非 active controller 的提交、非法 command、
  controller 主动报告 `COMMIT_ERROR` 或缺失 commit 超时，都会关闭当前 Tick、清空
  pending commits，并将 active controllers 置为 `ERROR`。身份正确但没有 open tick 的
  commit 只返回 `FAILED_PRECONDITION`。当前没有 resume/reset 方法；成功重载模型或
  重建 Core 才能回到可运行状态。
- `AcceptHeartbeat()` 只更新状态中的最后 heartbeat tick。当前
  `CheckTickTimeout()` 判断的是 open tick 后尚未提交 commit 的耗时，不使用 heartbeat
  年龄，也不会自行定时调用。
- `SimulationCore` 没有声明为线程安全；所有方法的并发访问都应由宿主串行化或加锁。

## 测试

从仓库根目录运行：

```bash
colcon build \
  --packages-up-to mujoco_sim_core \
  --cmake-args -DBUILD_TESTING=ON
colcon test \
  --packages-select mujoco_sim_core \
  --event-handlers console_direct+
colcon test-result \
  --test-result-base build/mujoco_sim_core \
  --verbose
```

[simulation_core_test.cpp](test/simulation_core_test.cpp) 当前覆盖默认与 keyframe 模型初始化、
缺失 keyframe 的原子失败、模型/资源目录、资源发现、快照隔离与刷新、注册冲突、激活、
Tick observation、barrier、重复 commit、非法值和 control range、session/epoch/tick 校验及
超时进入 `ERROR_PAUSED`。

## 相关包

- [仓库总览](../../Readme.md)
- [mujoco_sim_protocol](../mujoco_sim_protocol/Readme.md)：Core 使用的 V1 消息和命名约定
- [mujoco_sim_server](../mujoco_sim_server/Readme.md)：eCAL composition root、配置、自动 Tick 循环与 GUI
- [mujoco_sim_adapter_sdk](../mujoco_sim_adapter/mujoco_sim_adapter_sdk/Readme.md)：远端控制器接入 SDK
- [mujoco_sim_sample_adapter](../mujoco_sim_adapter/mujoco_sim_sample_adapter/Readme.md)：端到端控制器样例
