# 基于 colcon 的独立 MuJoCo 多 Adapter Lockstep 仿真平台方案

**文档版本：** 1.0  
**建议基线：** Ubuntu 24.04、ROS 2 Jazzy、CMake + Ninja、C++20（如需兼容更旧工具链可降为 C++17）  
**核心中间件：** MuJoCo + eCAL + Protobuf  

---

## 1. 方案摘要

本项目构建一个与 ROS 2 解耦的 MuJoCo 仿真平台。MuJoCo Core 是唯一物理状态所有者、唯一仿真时间源和唯一 `mjData` 写入者；ROS 2、强化学习策略、原生 C++ 控制器等功能均以独立 Adapter 进程接入。

系统采用同步 Tick 协议：

1. Core 发布 `Tick(n, state_n, time_n)`；
2. 所有当前 Tick 所需的同步 Adapter 并行执行控制更新；
3. 每个 Adapter 原子提交 `Commit(n, commands)`；
4. Core 等待 Barrier 满足，验证并合并命令；
5. Core 写入 `mjData->ctrl`，执行 `mj_step()`；
6. Core 发布 `Tick(n+1)`。

构建层面采用：

- **colcon：** 多包发现、依赖拓扑、增量构建、测试编排、install 环境聚合；
- **标准 CMake：** Protocol、Core、eCAL Transport、Adapter SDK、Server、工具与测试；
- **ament_cmake：** 仅 ROS 2 Adapter 和 ROS 2 Bringup；
- **package.xml：** 所有内部包的名称、版本和依赖关系的唯一声明来源；
- **colcon.pkg：** 仅用于无法修改的第三方项目或缺少 package manifest 的外部包；
- **colcon_defaults.yaml / colcon.meta：** 工作空间级和包级构建参数。

最终形成下面的依赖方向：

```text
mujoco_sim_protocol
        ↓
mujoco_sim_common
        ↓
┌───────────────────────┬────────────────────────┐
│                       │                        │
mujoco_sim_core   mujoco_sim_transport_api   mujoco_sim_model_resources
│                       │
│                 mujoco_sim_transport_ecal
│                       │
└───────────────┬───────┘
                ↓
        mujoco_sim_server
                ↑
        mujoco_sim_adapter_sdk
          ↑             ↑
 ROS 2 Adapter     Native/RL Adapter
```

依赖必须始终从 Adapter 指向 SDK/协议，不允许 Core 反向依赖 ROS 2、eCAL 业务代码或具体 Adapter。

---

## 2. 建设目标与边界

### 2.1 目标

平台应具备：

- MuJoCo Core 与 ROS 2 完全解耦；
- 支持多个同步 Adapter 和多个异步观察 Adapter；
- 支持严格 lockstep、单步、暂停、复位和不限速运行；
- 支持 Adapter 注册、资源声明、Barrier、超时和掉线处理；
- 支持 ROS 2 `/clock`、`ros2_control` 和标准传感器消息；
- 支持同机 eCAL Shared Memory 和跨机 eCAL TCP/UDP；
- 支持图像、点云等大数据独立数据面；
- 支持确定性回归测试和录制/回放；
- 支持按包增量构建，而不是每次重编整个工程；
- 支持无 ROS 环境下只构建 Core、Server 和原生 Adapter。

### 2.2 非目标

第一阶段不建议直接实现：

- Adapter 直接共享并修改 `mjData`；
- 多个 Adapter 通过到达顺序抢占同一个 actuator；
- 将所有可视化和 Recorder 纳入同步 Barrier；
- 第一版即实现 CUDA IPC；
- 在构建期间通过 FetchContent 在线下载所有依赖；
- 使用一个巨型顶层 `CMakeLists.txt` 管理整个仓库。

---

## 3. 系统总体架构

```text
┌─────────────────────────────────────────────────────────────┐
│                    MuJoCo Simulator Server                  │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ mujoco_sim_core                                       │  │
│  │                                                       │  │
│  │ MuJoCoBackend      SensorManager      ModelManager    │  │
│  │ TickScheduler      BarrierManager     AdapterRegistry │  │
│  │ CommandArbiter     SimulationClock    StateSnapshot   │  │
│  └───────────────────────────┬───────────────────────────┘  │
│                              │                              │
│  ┌───────────────────────────▼───────────────────────────┐  │
│  │ mujoco_sim_transport_ecal                            │  │
│  │ Registration Service / Tick / Commit / Sensor Topics │  │
│  └───────────────────────────┬───────────────────────────┘  │
└──────────────────────────────┼──────────────────────────────┘
                               │ eCAL
          ┌────────────────────┼──────────────────────┐
          │                    │                      │
          ▼                    ▼                      ▼
┌─────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ ROS 2 Adapter   │  │ Native Adapter   │  │ RL Adapter       │
│ /clock          │  │ C++ Controller   │  │ Python/C++       │
│ ros2_control    │  │                  │  │ Policy           │
└─────────────────┘  └──────────────────┘  └──────────────────┘
          │
          ▼
  RViz / TF / ROS Nodes
```

### 3.1 Core 的职责

Core 负责：

- 加载、重载和复位 MuJoCo 模型；
- 管理 `mjModel`、`mjData` 生命周期；
- 维护 `epoch`、`tick_id`、仿真时间和 episode 时间；
- 生成原子状态快照；
- 维护同步 Adapter 集合；
- 等待每 Tick Commit Barrier；
- 验证命令的 session、epoch、tick 和资源所有权；
- 按固定规则合并控制命令；
- 作为唯一实体写入 `mjData->ctrl`；
- 调用 `mj_step()`；
- 发布下一 Tick 和传感器数据。

Core 不包含：

- ROS 2 Node；
- `rclcpp`；
- `controller_manager`；
- ROS 消息类型；
- Adapter 专用控制逻辑；
- eCAL topic 名称硬编码之外的业务决策。

### 3.2 Server 的职责

`mujoco_sim_server` 是 Composition Root：

- 创建 Core；
- 创建 eCAL Transport；
- 将 Transport 事件转换为 Core API 调用；
- 将 Core 产生的 Tick、状态和事件发布到 eCAL；
- 加载 YAML 配置；
- 处理信号和进程退出；
- 提供日志、监控和诊断。

这样 Core 可以在单元测试中使用内存 Fake Transport，而无需启动 eCAL。

### 3.3 Adapter 的职责

同步 Adapter：

- 注册 Adapter 身份和资源声明；
- 接收 Tick；
- 读取该 Tick 的状态快照；
- 计算控制结果；
- 原子提交 Commit；
- 不直接修改 Core 或共享 MuJoCo 内存。

异步 Adapter：

- 只观察状态、图像、点云、诊断或日志；
- 不进入 Barrier；
- 处理速度不能影响物理步推进。

---

## 4. Lockstep 协议

### 4.1 Tick 语义

```text
Tick(n) 表示：
- 当前状态是 x_n；
- 当前全局仿真时间是 t_n；
- Adapter 应计算用于 x_n → x_(n+1) 的命令 u_n。
```

完整周期：

```text
Core:    publish Tick(n, x_n)
Adapter: compute u_n in parallel
Adapter: publish Commit(n, u_n)
Core:    wait required commits
Core:    validate + merge u_n
Core:    mj_step()
Core:    publish Tick(n+1, x_(n+1))
```

### 4.2 原子 Commit

禁止把控制数据与“更新完成”拆为两个独立消息。标准提交必须是：

```protobuf
message AdapterCommit {
  string adapter_id = 1;
  uint64 session_id = 2;
  uint64 epoch = 3;
  uint64 tick_id = 4;
  CommitStatus status = 5;
  repeated ActuatorCommandBlock commands = 6;
  string error_message = 7;
}
```

这使 `Commit` 同时表示：

- Adapter 已完成 Tick；
- 这就是该 Tick 的完整命令；
- Core 不会出现先收到 Done、后收到 Command 的竞态。

### 4.3 Barrier 参与者

Adapter 注册时声明：

```yaml
adapter_id: ros2_control
required_for_tick: true
update_divisor: 1
claimed_resources:
  - robot/left_arm/*
timeout_policy: fail_stop
```

Core 只等待当前 Tick 的 `required_for_tick` 参与者。Recorder、RViz、Foxglove、Metrics 等必须注册为 Observer。

### 4.4 多频率 Adapter

以 1 kHz 基础 Tick 为例：

```yaml
ros2_control:
  update_divisor: 1     # 1000 Hz
rl_policy:
  update_divisor: 10    # 100 Hz
slow_planner:
  update_divisor: 100   # 10 Hz
```

Core 根据 `tick_id % update_divisor == 0` 决定本 Tick 是否等待该 Adapter。非更新 Tick 的命令保持策略必须显式配置：

- `HOLD_LAST`；
- `ZERO`；
- `INTERPOLATE`；
- `CORE_DEFAULT_CONTROLLER`。

### 4.5 资源所有权和命令仲裁

默认规则：一个 actuator 只能由一个同步 Adapter 独占。

确实需要重叠时，必须配置确定性仲裁：

- 固定优先级覆盖；
- 明确定义的增量相加；
- 控制模式切换；
- Safety Adapter 最终裁剪。

严禁根据消息到达先后决定最终命令。

### 4.6 可靠性

高频 Tick/Commit 可以继续使用 eCAL Pub/Sub，但应用层必须实现：

- 当前 Tick 周期性重发；
- Commit 周期性重发；
- `(adapter_id, session_id, epoch, tick_id)` 幂等去重；
- steady clock 超时；
- 过期、未来和旧 session 消息拒绝；
- 当前 Tick 完成后停止重发。

注册、配置、暂停、复位和加载模型使用 eCAL Service。

对于未来严格跨机控制，可将 Tick/Commit 控制面替换为持久 TCP/Unix Domain Socket，状态和图像数据面仍使用 eCAL；协议层和 Core 不需要改变。

---

## 5. 时间模型与 ROS 2 同步

### 5.1 时间字段

每个协议头统一包含：

```protobuf
message SimulationHeader {
  uint64 epoch = 1;
  uint64 tick_id = 2;
  uint64 global_sim_time_ns = 3;
  uint64 episode_time_ns = 4;
  uint64 timestep_ns = 5;
  uint64 sequence = 6;
  uint64 source_steady_time_ns = 7;
}
```

语义：

- `global_sim_time_ns`：单调不回退，用于 ROS `/clock` 和所有 ROS Header；
- `episode_time_ns`：Reset 后回到 0，用于任务和 RL；
- `tick_id`：离散物理步编号；
- `epoch`：模型重载、世界重置或协议代次；
- `source_steady_time_ns`：性能统计和 wall timeout，不作为 ROS 时间。

### 5.2 ROS 2 Adapter

收到 `Tick(n)` 后，ROS Adapter 在自己的 Worker Thread 中执行：

```text
publish /clock(t_n)
update ros2_control state interfaces
controller_manager.read(t_n, dt)
controller_manager.update(t_n, dt)
controller_manager.write(t_n, dt)
collect commands
publish Commit(n, commands)
```

所有 ROS 节点设置：

```yaml
use_sim_time: true
```

ROS Adapter 必须使用 Tick 中的仿真时间填充消息时间戳，不能使用 Adapter 的接收时刻。

### 5.3 Pause、Single Step 和 Reset

- Pause：不执行 `mj_step()`，仿真时间不推进；可低频重复发布当前 `/clock`；
- Single Step：完成一个完整 Barrier 和一个 `mj_step()` 后再次暂停；
- Reset：中止或完成当前事务，`epoch += 1`，清空 Commit 和控制缓存，复位 Adapter 控制器状态；
- 推荐全局 ROS 时间持续单调，episode 时间单独归零。

---

## 6. colcon 构建策略

### 6.1 colcon 的定位

colcon 是多包构建编排工具，不替代 CMake。它负责：

- 发现工作空间中的包；
- 根据 manifest 建立依赖图；
- 按拓扑顺序构建；
- 为每个包建立独立 build 目录；
- 安装到 install 空间；
- 聚合环境；
- 选择性构建和测试。

项目仍然要求每个 C++ 包是一个规范、可独立安装和被 `find_package()` 使用的 CMake Package。

### 6.2 包类型

| 模块 | colcon 类型 | 构建系统 |
|---|---|---|
| Protocol、Common、Core | `ros.cmake`/CMake | 标准 CMake |
| eCAL Transport、Adapter SDK | CMake | 标准 CMake |
| Server、CLI、Tests | CMake | 标准 CMake + CTest |
| ROS 2 Adapter | `ros.ament_cmake` | ament_cmake |
| ROS 2 Bringup | `ros.ament_cmake` | ament_cmake |
| Python Adapter | Python | `ament_python` 或标准 Python 扩展 |

推荐所有内部包都提供 `package.xml`。纯 CMake 包在 `package.xml` 中声明：

```xml
<export>
  <build_type>cmake</build_type>
</export>
```

ROS 2 CMake 包声明：

```xml
<export>
  <build_type>ament_cmake</build_type>
</export>
```

### 6.3 为什么不使用一个顶层 CMake

单一顶层 CMake 会导致：

- Core 与 ROS 2 Adapter 绑定在同一配置阶段；
- 很难只构建 Adapter SDK 或 Protocol；
- 第三方依赖污染所有目标；
- 安装导出和 ABI 边界不清晰；
- 单包测试、缓存和发布困难。

colcon 工作空间中每个包必须拥有自己的：

```text
package.xml
CMakeLists.txt
include/<package>/...
src/...
test/...
cmake/...
```

---

## 7. 推荐仓库目录

```text
mujoco_sim_ws/
├── README.md
├── LICENSE
├── colcon_defaults.yaml
├── colcon.meta
├── dependencies.repos
├── config/
│   ├── simulator.yaml
│   ├── adapters.yaml
│   ├── ecal.yaml
│   └── colcon/
│       ├── dev.yaml
│       ├── release.yaml
│       ├── asan.yaml
│       └── core-only.yaml
├── scripts/
│   ├── bootstrap.sh
│   ├── build.sh
│   ├── test.sh
│   ├── run_simulator.sh
│   └── run_ros2_adapter.sh
├── docker/
│   ├── Dockerfile.dev
│   ├── Dockerfile.ci
│   └── compose.yaml
├── src/
│   ├── mujoco_sim_protocol/
│   ├── mujoco_sim_common/
│   ├── mujoco_sim_transport_api/
│   ├── mujoco_sim_transport_ecal/
│   ├── mujoco_sim_core/
│   ├── mujoco_sim_server/
│   ├── mujoco_sim_adapter_sdk/
│   ├── mujoco_sim_model_resources/
│   ├── mujoco_sim_ros2_control/
│   ├── mujoco_sim_ros2_bringup/
│   ├── mujoco_sim_examples/
│   ├── mujoco_sim_tools/
│   └── mujoco_sim_integration_tests/
├── build/                 # colcon 生成，不提交
├── install/               # colcon 生成，不提交
└── log/                   # colcon 生成，不提交
```

`.gitignore`：

```gitignore
/build/
/install/
/log/
.cache/
compile_commands.json
*.user
```

需要临时排除目录时，在目录中创建：

```text
COLCON_IGNORE
```

---

## 8. 包职责与依赖

### 8.1 `mujoco_sim_protocol`

职责：

- 保存所有 `.proto`；
- 生成并安装 C++ Protobuf 库；
- 定义版本、Header、Tick、Commit、注册、错误、状态和传感器消息；
- 不依赖 MuJoCo、eCAL 或 ROS 2。

目录：

```text
mujoco_sim_protocol/
├── package.xml
├── CMakeLists.txt
├── cmake/mujoco_sim_protocolConfig.cmake.in
├── proto/mujoco/sim/v1/
│   ├── common.proto
│   ├── control.proto
│   ├── lifecycle.proto
│   ├── state.proto
│   └── sensor.proto
└── test/
```

协议规则：

- 只新增字段，不复用已删除 field number；
- 删除字段必须 `reserved`；
- 包名带主版本，如 `mujoco.sim.v1`；
- 协议生成代码只在 build 目录产生；
- 安装 `.proto` 原文件，供 Python/其他语言 Adapter 生成绑定。

### 8.2 `mujoco_sim_common`

职责：

- 强类型 ID；
- 时间和 Tick 类型；
- Status/Expected；
- 日志适配接口；
- 配置结构；
- 不包含 Transport 和 MuJoCo 对象。

### 8.3 `mujoco_sim_transport_api`

定义抽象接口：

```cpp
class ControlTransport {
public:
  virtual void publish_tick(const TickFrame&) = 0;
  virtual void publish_event(const SimulationEvent&) = 0;
  virtual std::optional<AdapterCommit> wait_commit(Deadline) = 0;
  virtual ~ControlTransport() = default;
};
```

该包只依赖 Protocol/Common，方便 Core 测试使用 Fake Transport。

### 8.4 `mujoco_sim_transport_ecal`

职责：

- eCAL 初始化和配置；
- Topic/Service 封装；
- Protobuf 序列化；
- Tick/Commit 重传；
- 消息去重；
- Subscriber callback 快速入队；
- 图像 Binary API；
- 诊断统计。

只允许该包直接包含 eCAL 头文件。Core 和 ROS Adapter 不应直接调用 eCAL API。

### 8.5 `mujoco_sim_core`

核心对象建议：

```text
SimulationCore
MuJoCoBackend
TickScheduler
BarrierManager
AdapterRegistry
CommandArbiter
SimulationClock
StateSnapshotBuilder
SensorManager
ModelManager
```

Core 对外提供同步 API，不自行创建 eCAL Topic。

### 8.6 `mujoco_sim_server`

可执行文件：

```text
mujoco-sim-server
```

依赖：

```text
core + transport_ecal + protocol + model_resources
```

### 8.7 `mujoco_sim_adapter_sdk`

提供 Adapter 开发者 API：

```cpp
class TickAdapter {
public:
  RegistrationResult register_adapter(const AdapterSpec&);
  TickFrame wait_tick();
  CommitResult commit(const TickContext&, CommandSet);
  void heartbeat();
};
```

SDK 隐藏：

- eCAL Topic；
- Protobuf 细节；
- 重传和去重；
- session 管理；
- epoch/tick 校验；
- 心跳线程。

### 8.8 `mujoco_sim_ros2_control`

职责：

- 初始化 ROS 2 Context；
- 作为 Adapter 注册；
- 发布 `/clock`；
- 暴露 ros2_control state/command interfaces；
- 由 Tick 驱动 `read-update-write`；
- 发布标准 ROS 传感器和 TF；
- 将 command interface 转换为 Commit。

该包不得链接 MuJoCo。

### 8.9 `mujoco_sim_model_resources`

安装：

- MJCF；
- mesh；
- texture；
-默认 world；
- sensor/actuator mapping；
- 示例配置。

资源通过 install prefix 查找，不能依赖源码绝对路径。

---

## 9. CMake 包模板

### 9.1 标准纯 CMake 库

```cmake
cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_common VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

add_library(${PROJECT_NAME}
  src/time.cpp
  src/status.cpp
)
add_library(mujoco_sim::common ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE mujoco_sim::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
```

`Config.cmake.in`：

```cmake
@PACKAGE_INIT@
include("${CMAKE_CURRENT_LIST_DIR}/mujoco_sim_commonTargets.cmake")
check_required_components(mujoco_sim_common)
```

### 9.2 Protocol 包

```cmake
cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_protocol VERSION 1.0.0 LANGUAGES CXX)

find_package(Protobuf REQUIRED)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(PROTO_FILES
  proto/mujoco/sim/v1/common.proto
  proto/mujoco/sim/v1/control.proto
  proto/mujoco/sim/v1/lifecycle.proto
  proto/mujoco/sim/v1/state.proto
  proto/mujoco/sim/v1/sensor.proto
)

add_library(${PROJECT_NAME})
add_library(mujoco_sim::protocol ALIAS ${PROJECT_NAME})

protobuf_generate(
  TARGET ${PROJECT_NAME}
  LANGUAGE cpp
  PROTOS ${PROTO_FILES}
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated
)

target_link_libraries(${PROJECT_NAME} PUBLIC protobuf::libprotobuf)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# 其余 install/export 与普通 CMake 库相同
```

### 9.3 Core 查找 MuJoCo

```cmake
find_package(mujoco CONFIG REQUIRED)
find_package(mujoco_sim_common CONFIG REQUIRED)
find_package(mujoco_sim_protocol CONFIG REQUIRED)
find_package(mujoco_sim_transport_api CONFIG REQUIRED)

target_link_libraries(mujoco_sim_core
  PUBLIC
    mujoco_sim::common
    mujoco_sim::protocol
    mujoco_sim::transport_api
  PRIVATE
    mujoco::mujoco
)
```

### 9.4 eCAL Transport

```cmake
find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)
find_package(mujoco_sim_protocol CONFIG REQUIRED)
find_package(mujoco_sim_transport_api CONFIG REQUIRED)

target_link_libraries(mujoco_sim_transport_ecal
  PUBLIC
    mujoco_sim::transport_api
    mujoco_sim::protocol
  PRIVATE
    eCAL::core
    protobuf::libprotobuf
)
```

### 9.5 ROS 2 Adapter

```cmake
cmake_minimum_required(VERSION 3.22)
project(mujoco_sim_ros2_control LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mujoco_sim_adapter_sdk CONFIG REQUIRED)

add_executable(mujoco_sim_ros2_adapter
  src/main.cpp
  src/ros2_adapter.cpp
  src/sim_system_interface.cpp
)

target_compile_features(mujoco_sim_ros2_adapter PRIVATE cxx_std_20)
ament_target_dependencies(mujoco_sim_ros2_adapter
  rclcpp
  rosgraph_msgs
  sensor_msgs
  controller_manager
  hardware_interface
  pluginlib
)

target_link_libraries(mujoco_sim_ros2_adapter
  PRIVATE mujoco_sim::adapter_sdk
)

install(TARGETS mujoco_sim_ros2_adapter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## 10. package.xml 模板

### 10.1 纯 CMake 包

```xml
<?xml version="1.0"?>
<package format="3">
  <name>mujoco_sim_core</name>
  <version>0.1.0</version>
  <description>Transport-independent MuJoCo simulation core.</description>
  <maintainer email="team@example.com">Simulation Team</maintainer>
  <license>Apache-2.0</license>

  <depend>mujoco_sim_common</depend>
  <depend>mujoco_sim_protocol</depend>
  <depend>mujoco_sim_transport_api</depend>

  <test_depend>gtest</test_depend>

  <export>
    <build_type>cmake</build_type>
  </export>
</package>
```

系统依赖 MuJoCo、eCAL、Protobuf 由开发镜像、系统包或依赖 underlay 提供；内部包依赖必须完整写入 manifest。

### 10.2 ROS 2 Adapter

```xml
<?xml version="1.0"?>
<package format="3">
  <name>mujoco_sim_ros2_control</name>
  <version>0.1.0</version>
  <description>ROS 2 and ros2_control adapter for the MuJoCo simulator.</description>
  <maintainer email="team@example.com">Simulation Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rosgraph_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>controller_manager</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>mujoco_sim_adapter_sdk</depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>launch_testing</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 11. colcon 配置

### 11.1 `colcon_defaults.yaml`

开发默认配置：

```yaml
build:
  symlink-install: true
  executor: parallel
  parallel-workers: 8
  event-handlers:
    - console_cohesion+
  cmake-args:
    - -G
    - Ninja
    - -DCMAKE_BUILD_TYPE=RelWithDebInfo
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    - -DBUILD_TESTING=ON

test:
  event-handlers:
    - console_cohesion+
```

colcon 会为每个包创建独立 build 目录，并在启用 `CMAKE_EXPORT_COMPILE_COMMANDS` 时生成包级和聚合的编译数据库。

### 11.2 `colcon.meta`

只存包级差异，不复制通用参数：

```yaml
names:
  mujoco_sim_server:
    cmake-args:
      - -DMUJOCO_SIM_ENABLE_RENDERING=ON

  mujoco_sim_integration_tests:
    cmake-args:
      - -DMUJOCO_SIM_ENABLE_STRESS_TESTS=OFF
```

### 11.3 `colcon.pkg` 的使用原则

正常内部包不需要 `colcon.pkg`，因为 `package.xml` 已经能声明包名、类型和依赖。

`colcon.pkg` 只用于：

- 第三方纯 CMake 工程没有 `package.xml`；
- 不希望修改上游仓库；
- 需要为单个第三方包指定额外依赖或 CMake 参数；
- 需要声明额外环境 hook。

示例：

```yaml
name: external_example
type: cmake
dependencies:
  - mujoco_sim_common
cmake-args:
  - -DEXAMPLE_BUILD_TESTS=OFF
```

不要同时在 `package.xml` 和 `colcon.pkg` 重复维护内部依赖，否则容易出现依赖漂移。

---

## 12. 依赖管理

### 12.1 推荐基线

第一阶段建议：

```text
OS:       Ubuntu 24.04
ROS 2:    Jazzy
Compiler: GCC 13 / Clang
Generator:Ninja
MuJoCo:   固定版本，由系统或 /opt 前缀安装
eCAL:     固定 v6.x 版本，由系统或 /opt 前缀安装
Protobuf: 与 eCAL 和项目生成代码兼容的固定版本
```

### 12.2 Underlay 与 Overlay

推荐分层：

```text
/opt/ros/jazzy               ROS 2 underlay
/opt/mujoco_sim_deps         MuJoCo/eCAL/Protobuf underlay
~/mujoco_sim_ws/install      项目 overlay
```

构建前：

```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=/opt/mujoco_sim_deps:${CMAKE_PREFIX_PATH}
```

然后构建项目工作空间。

### 12.3 不建议在业务包中 FetchContent

原因：

- 每个包独立配置时可能重复下载；
- 离线构建不可控；
- CI 缓存困难；
- 版本锁定分散；
- 交叉编译和补丁管理困难。

依赖版本应固定在：

- Docker image；
- bootstrap 脚本；
- `dependencies.repos`；
- 独立 vendor/underlay workspace。

### 12.4 可选 vendor workspace

确实需要源码构建依赖时，建立独立：

```text
mujoco_sim_deps_ws/
├── src/
│   ├── ecal/
│   └── mujoco_vendor/
├── build/
├── install/
└── log/
```

先构建 deps workspace，再 source 它并构建主 workspace。不要让第三方大型项目与业务包在同一个日常增量构建中反复配置。

---

## 13. 日常构建流程

### 13.1 环境初始化

```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=/opt/mujoco_sim_deps:${CMAKE_PREFIX_PATH}
cd ~/mujoco_sim_ws
```

### 13.2 检查包发现和依赖

```bash
colcon list
colcon list --topological-order
colcon info mujoco_sim_server
colcon graph
```

### 13.3 全量开发构建

```bash
colcon build
source install/setup.bash
```

### 13.4 只构建 Core

```bash
colcon build \
  --packages-up-to mujoco_sim_core
```

### 13.5 构建 Server 及其依赖

```bash
colcon build \
  --packages-up-to mujoco_sim_server
```

### 13.6 只构建 ROS Adapter

```bash
colcon build \
  --packages-up-to mujoco_sim_ros2_control
```

### 13.7 修改 Protocol 后重建受影响包

```bash
colcon build \
  --packages-above-and-dependencies mujoco_sim_protocol
```

若本机 colcon 扩展不提供该组合参数，则用 `colcon graph` 确认影响范围，或显式选择协议包及其依赖包。

### 13.8 Release 构建

```bash
rm -rf build install log
COLCON_DEFAULTS_FILE=config/colcon/release.yaml colcon build
```

Release 配置建议：

```yaml
build:
  merge-install: true
  executor: parallel
  cmake-args:
    - -G
    - Ninja
    - -DCMAKE_BUILD_TYPE=Release
    - -DBUILD_TESTING=OFF
```

日常开发优先使用默认 isolated install，发布 staging 可以使用 merged install。isolated install 更容易发现漏写依赖；merged install 更适合制作紧凑运行包。

---

## 14. 运行配置

### 14.1 Simulator 配置

```yaml
instance: robot_lab

model:
  path: share/mujoco_sim_model_resources/worlds/robot.xml

simulation:
  timestep_ns: 1000000
  real_time_factor: 0.0
  start_paused: true
  clock_mode: global_monotonic

barrier:
  tick_timeout_ms: 100
  tick_resend_period_ms: 2
  commit_resend_period_ms: 2
  default_timeout_policy: fail_stop

command:
  reject_unclaimed_resources: true
  reject_duplicate_ownership: true
  validate_finite_values: true

sensors:
  control_state_rate_divisor: 1
  camera_rate_divisor: 33
```

### 14.2 Adapter allowlist

```yaml
adapters:
  ros2_control:
    required: true
    update_divisor: 1
    timeout_ms: 100
    timeout_policy: fail_stop
    resources:
      - robot/joint/*

  rl_policy:
    required: false
    update_divisor: 10
    timeout_ms: 500
    timeout_policy: hold_last
    resources:
      - robot/gripper/*
```

运行时注册仍要携带 claims；Core 根据配置 allowlist 验证。

### 14.3 Topic 命名

```text
mujoco_sim/<instance>/control/tick
mujoco_sim/<instance>/control/commit/<adapter_id>
mujoco_sim/<instance>/control/event
mujoco_sim/<instance>/state/frame
mujoco_sim/<instance>/sensor/imu/<name>
mujoco_sim/<instance>/sensor/camera/<name>/image
mujoco_sim/<instance>/diagnostics
```

Service：

```text
mujoco_sim/<instance>/register_adapter
mujoco_sim/<instance>/unregister_adapter
mujoco_sim/<instance>/pause
mujoco_sim/<instance>/resume
mujoco_sim/<instance>/single_step
mujoco_sim/<instance>/reset
mujoco_sim/<instance>/load_model
```

---

## 15. 线程模型

### 15.1 Server

建议至少划分：

```text
Main/Lifecycle Thread
Simulation Thread
Control Transport RX Thread
Control Transport TX/Resend Thread
Sensor Worker Pool
Render Thread
Diagnostics Thread
```

只有 Simulation Thread 可以访问和修改 `mjData`。其他线程通过队列提交事件。

### 15.2 ROS Adapter

```text
eCAL callback thread:
  校验和入队，立即返回

Tick worker thread:
  读取 Tick
  发布 /clock
  更新 state interface
  controller_manager read/update/write
  生成 Commit

ROS executor thread(s):
  参数、Service、非同步 ROS topic

Heartbeat thread:
  使用 steady clock
```

严格控制循环不得在普通 eCAL callback 中直接执行。

---

## 16. 测试方案

### 16.1 单元测试

`mujoco_sim_protocol`：

- 兼容性 golden files；
- serialize/deserialize；
- 删除字段 reserved 检查。

`mujoco_sim_core`：

- Barrier 收齐；
- 重复 Commit 去重；
- 旧 session 拒绝；
- 旧 epoch/tick 拒绝；
- timeout policy；
- actuator ownership；
- deterministic merge；
- pause/reset/single-step；
- global time 单调。

`mujoco_sim_transport_ecal`：

- 断连和恢复；
- Tick 重发；
- Commit 重发；
- callback 不阻塞；
- 大消息和共享内存配置。

### 16.2 集成测试

必须覆盖：

1. Core + 两个同步 Fake Adapter；
2. 两 Adapter 控制不同 actuator；
3. Adapter 超时导致 fail-stop；
4. hold-last 限制最大 command age；
5. Adapter 崩溃重启，旧 session 消息被拒绝；
6. Reset 后旧 epoch Commit 被丢弃；
7. ROS Adapter `/clock` 与 Tick 一致；
8. 1000 个 Tick 后状态哈希可重现；
9. 录制并回放相同 Commit 序列得到相同状态；
10. 多 Observer 不影响 Tick 吞吐。

### 16.3 colcon 测试

```bash
colcon test
colcon test-result --verbose
```

选择测试：

```bash
colcon test --packages-select mujoco_sim_core
colcon test --packages-select mujoco_sim_integration_tests
```

纯 CMake 包使用 CTest，ROS 包使用 ament test/launch_testing；colcon 统一调度。

### 16.4 Sanitizer

ASan/UBSan profile：

```yaml
build:
  cmake-args:
    - -G
    - Ninja
    - -DCMAKE_BUILD_TYPE=Debug
    - -DBUILD_TESTING=ON
    - -DCMAKE_CXX_FLAGS=-fsanitize=address,undefined\ -fno-omit-frame-pointer
    - -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined
```

TSan 单独运行，不与 ASan 混合。

---

## 17. CI/CD

建议流水线：

```text
lint
  ↓
protocol-build
  ↓
core-build-and-unit-test
  ↓
ecal-transport-test
  ↓
server-integration-test
  ↓
ros2-adapter-build-and-test
  ↓
determinism-test
  ↓
release-stage
```

### 17.1 CI 镜像

镜像固定：

- Ubuntu；
- ROS 2 发行版；
- eCAL；
- MuJoCo；
- Protobuf；
- CMake/Ninja；
- GCC/Clang；
- colcon 扩展版本。

不要在每次 CI 中获取“最新版本”。

### 17.2 缓存

缓存：

- compiler cache；
- 第三方依赖 install prefix；
- 可选择缓存 colcon build，但必须把编译器、依赖版本和 CMake 参数纳入 key。

### 17.3 质量门槛

合并前至少要求：

- 所有包可从空 build/install 构建；
- `colcon test-result` 无失败；
- Core 无 ROS 2/eCAL 直接依赖；
- Protocol 兼容性测试通过；
- 两 Adapter lockstep 集成测试通过；
- deterministic hash 测试通过；
- ASan/UBSan 通过。

---

## 18. 发布与部署

colcon 负责构建和安装，不等价于最终软件包管理器。

建议发布流程：

1. 在固定 CI 镜像执行干净 Release 构建；
2. 使用 merged install 生成 staging；
3. 安装配置、模型和 systemd 文件；
4. 进行无源码运行测试；
5. 输出 tar 包、容器镜像或 Debian 包；
6. 记录 Protocol、Core、eCAL、MuJoCo 和 ROS 2 版本。

运行组件可以拆成：

```text
mujoco-sim-runtime
mujoco-sim-dev
mujoco-sim-ros2-adapter
mujoco-sim-models
mujoco-sim-tools
```

非 ROS 机器只安装 runtime、models 和所需 Adapter。

---

## 19. 分阶段实施计划

### Phase 0：仓库与构建骨架

- 建立 colcon workspace；
- 创建 Protocol、Common、Core、Transport API 空包；
- 完成标准 CMake install/export；
- 验证 `colcon list`、拓扑和选择性构建；
- 建立 CI 干净构建。

验收：每个包均可安装，并被下游 `find_package()`。

### Phase 1：单 Adapter Lockstep

- Core MuJoCoBackend；
- TickScheduler；
- eCAL Tick/Commit；
- 一个 Native Adapter；
- 单步、暂停、复位；
- 基本 deterministic test。

### Phase 2：多 Adapter Barrier

- AdapterRegistry；
- session/epoch/tick；
- BarrierManager；
- 资源声明；
- CommandArbiter；
- timeout policy；
- 重传和去重。

### Phase 3：ROS 2 Adapter

- `/clock`；
- Tick 驱动 controller_manager；
- state/command interface；
- joint state、IMU、TF；
- launch_testing。

### Phase 4：大数据与工具

- camera/depth Binary API；
- eCAL Monitor/Recorder 配置；
- Foxglove/Recorder Observer；
- 性能指标；
- 图像多缓冲。

### Phase 5：生产化

- Docker/CI；
- ABI/Protocol 版本策略；
- 发布 staging；
- 跨机测试；
- 故障注入；
- 可选 CUDA IPC。

---

## 20. 关键工程约束

以下约束应写入项目 Architecture Decision Records：

1. MuJoCo Core 是唯一时间源和唯一 `mjData` 写入者。
2. Adapter 永远不直接写共享 MuJoCo 状态。
3. 控制命令和完成状态必须原子 Commit。
4. Barrier 只等待显式注册的同步参与者。
5. 所有控制消息包含 `session_id + epoch + tick_id`。
6. 超时使用 steady wall clock，不使用仿真时间。
7. ROS `/clock` 使用 Core 提供的全局仿真时间。
8. Core 不依赖 ROS 2。
9. eCAL 只出现在 Transport 包和 Adapter SDK 的私有实现中。
10. 所有 CMake 库必须安装并导出 Config Package。
11. 所有内部依赖必须在 `package.xml` 中声明。
12. `colcon.pkg` 不作为内部包依赖的第二份真相。
13. 开发构建默认 isolated install；发布 staging 可 merged install。
14. 所有 actuator 冲突必须在 Tick 前确定，不能靠消息到达顺序仲裁。
15. Protocol 字段号永不复用。

---

## 21. 最终落地选择

建议第一版直接采用：

```text
Build orchestration: colcon
Core build:          standard CMake
ROS package build:   ament_cmake
C++ standard:        C++20
Native generator:    Ninja
Serialization:       Protobuf
IPC/data transport:  eCAL
Control protocol:    Tick + atomic Commit + Barrier
Time authority:      MuJoCo Core
ROS time:            /clock from global_sim_time_ns
Dependency strategy: pinned system/deps underlay
```

最小可运行包集合：

```text
mujoco_sim_protocol
mujoco_sim_common
mujoco_sim_transport_api
mujoco_sim_transport_ecal
mujoco_sim_core
mujoco_sim_server
mujoco_sim_adapter_sdk
mujoco_sim_examples
```

在这个集合稳定后，再加入：

```text
mujoco_sim_ros2_control
mujoco_sim_ros2_bringup
mujoco_sim_model_resources
mujoco_sim_integration_tests
```

该结构可以保证：仿真 Core 不被 ROS 2 绑定，多 Adapter 可以严格同步，colcon 能按依赖增量构建，且每个模块仍保持标准 CMake 工程的独立可用性。

---

## 22. 官方依据

本文方案依据以下官方资料中的能力边界设计：

- colcon 官方文档：工作空间、多包拓扑构建、选择性构建、isolated/merged install、`colcon.pkg`、`.meta`、`colcon_defaults.yaml`、CTest 和 compile commands；
- ROS 2 官方文档：ament_cmake、package.xml、overlay/underlay、ROS 2 Jazzy 对 Ubuntu 24.04 的支持；
- eCAL 官方项目和文档：Pub/Sub、Service、同机 Shared Memory、网络 UDP/TCP、CMake target `eCAL::core`；
- MuJoCo 官方源码构建配置：安装后 CMake target `mujoco::mujoco`；
- Protobuf 官方文档：C++ 生成代码、跨语言消息和字段兼容性最佳实践。
