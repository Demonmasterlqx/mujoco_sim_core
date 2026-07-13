# MuJoCo Sim Core V1 Handoff

## 1. 文档目的

本文面向下一位维护者，说明截至 2026-07-12 仓库已经实现的能力、当前架构、运行时状态机、
仿真循环、测试基线、已知限制和建议的后续工作。详细验收记录仍以同目录下的
[`v1.plan`](./v1.plan) 为准；本文侧重于“接手后如何理解和继续开发”。

当前 V1 基线提交：

```text
5a7e8cd3f78406883ea7e72c3c1ad5b27fd467b8
feat: add sensor observers and streams
```

该提交完成了动态 Controller、同步 Observer、异步 Sensor Stream、GUI/headless Server、
配置生成、状态工具、WBR LQR Demo 和 Sensor/Mapping Demo。本文编写期间，除本文档外的
工作区状态还包含：

```text
M  demos/mujoco_sim_sensor_demo/config/sensor_demo_server.yaml
M  demos/mujoco_sim_wbr_lqr_demo/test/app_config_test.cpp
?? sensor_demo_output/
```

这些内容不是本文档产生的改动。接手时不要擅自恢复或覆盖；提交前应分别确认配置和 WBR
测试改动的归属，并确认 `sensor_demo_output/` 是否为仍需保留的运行结果，不能直接混入架构
修改提交。

## 2. 当前完成状态

V1 已经具备一个可运行的 MuJoCo lockstep 仿真系统：

- Server 是唯一的 `mjModel`/`mjData` 主状态持有者、唯一 `mjData.ctrl` 写入者和唯一
  `mj_step()` 调用者。
- Server 加载模型后立即进入 `RUNNING`，没有 Adapter 时也按模型 timestep 自主推进。
- Controller 可以在运行期注册、激活、注销，通过 Tick/Commit Barrier 参与同步控制。
- 单个 Controller 超时、命令错误或注销不会令整个 Server 退出。
- Observer 可以按仿真 step 或 simulation-time period 同步读取低带宽具名 MuJoCo Sensor，
  但不提交命令、不进入 Barrier。
- Sensor Stream 可以异步发布 RGB、深度、分割、相机点云和 360 度多线 LiDAR。
- GUI 使用 MuJoCo 官方 `simulate` UI；`--headless` 不创建窗口，但配置了相机 Stream 时仍会
  在 Sensor worker 中初始化 EGL/OSMesa 离屏 OpenGL。
- 所有正式配置入口使用 `mujoco_sim_config` 生成的 CLI11 + yaml-cpp 解析代码，优先级固定为
  CLI 显式值 > YAML > C++ 默认值。
- 全部核心能力都有单元、安装态 consumer 或真实 eCAL 端到端测试，并完成多轮 subagent
  只读审查。

最终 clean 验收结果：

```text
9 packages built
50 tests, 0 errors, 0 failures, 0 skipped
```

## 3. 包与依赖关系

`colcon list --topological-order` 当前包含 9 个纯 CMake/colcon 包：

| 包 | 主要职责 | 主要下游 |
| --- | --- | --- |
| `mujoco_sim_config` | 从 schema 生成 C++20 DTO、CLI11 声明、严格 YAML 加载和帮助文本 | Server、状态工具、Sample Adapter、两个 Demo |
| `mujoco_sim_protocol` | Protobuf wire contract、topic/service 命名、raw stream framing | Core、Server、SDK、应用 |
| `mujoco_sim_core` | MuJoCo 主状态、Controller/Observer 状态机、Barrier、step、快照 | Server、Sensor、模型 contract 测试 |
| `mujoco_sim_sensor` | transport-free 相机和 LiDAR 捕获、Stream descriptor | Server、Sensor Demo |
| `mujoco_sim_server` | composition root、eCAL transport、仿真循环、SensorRuntime、GUI、日志、状态工具 | 可执行 Server、Demo |
| `mujoco_sim_adapter_sdk` | Controller、Observer、Stream 三类客户端和共享 eCAL runtime | Sample Adapter、两个 Demo |
| `mujoco_sim_sample_adapter` | 最小 Controller 示例 | E2E 验收 |
| `mujoco_sim_wbr_lqr_demo` | 迁移的 WBR LQR 同步控制示例 | Headless/GUI E2E |
| `mujoco_sim_sensor_demo` | 四轮车、相机、IMU、LiDAR、ICP 建图完整示例 | Headless/GUI/质量门禁 E2E |

依赖方向必须保持单向：

```text
config -------------------------------> applications

protocol ---> core ---> server
    |           |         |
    |           +-> sensor+
    |                     |
    +-----------------> adapter_sdk ---> adapters/demos
```

关键边界：

- `mujoco_sim_protocol` 不依赖 MuJoCo、eCAL 或业务实现。
- `mujoco_sim_core` 不依赖 eCAL，不创建 service/topic。
- `mujoco_sim_sensor` 不依赖 eCAL，不拥有线程、调度或队列。
- eCAL endpoint、线程编排和进程生命周期集中在 Server 或 Adapter SDK。
- 算法代码应保持 transport-free，进程入口负责把 SDK 数据转换成算法输入。

## 4. 运行时进程拓扑

典型部署中只有一个 Server，可以有任意数量的 Controller、Observer 和 Stream consumer：

```text
                         registry service
                  +----------------------------+
                  |                            v
+------------+    |   Tick/Commit/Heartbeat +------------------+
| Controller |<---+-------------------------->|                  |
+------------+                               |                  |
                                               | mujoco-sim-server|
+------------+       ObservationFrame        |                  |
| Observer   |<------------------------------| Core + Barrier   |
+------------+        Observer heartbeat      | SensorRuntime    |
                  --------------------------->| eCAL transport   |
                                               | GUI or headless  |
+------------+          raw Stream frame       |                  |
| Stream     |<------------------------------|                  |
| consumer   |        ServerStatus/catalog    +------------------+
+------------+
```

同一进程也可以组合多种 Client。例如 Sensor Demo mapper 同时持有 `ObserverClient` 和
`SensorStreamClient`。SDK 使用进程级 eCAL runtime lease，任何一个 Client 的销毁都不会提前
`eCAL::Finalize()` 破坏同进程其他 Client。

## 5. 核心所有权与并发模型

### 5.1 MuJoCo 主状态

Server 内的 Core 独占真实 `mjModel` 和 `mjData`。所有会读取或修改主状态的 Core API 都在
Server 状态锁保护下调用。禁止 eCAL callback 直接访问 MuJoCo。

控制命令的唯一写入路径是：

```text
Commit validation
  -> ownership/layout/range validation
  -> completed Barrier
  -> write selected actuator values to mjData.ctrl
  -> mj_step()
```

### 5.2 快照

GUI 和 Sensor worker 不持有主状态指针：

- GUI 使用隔离的 model/data 深拷贝快照。
- Sensor worker 使用共享只读 model、独占 data 的 snapshot pool。
- Core 在锁内把 completed-step 状态复制到空闲 snapshot。
- 渲染、射线计算、序列化和 eCAL publish 在仿真锁外执行。

Snapshot clone 必须复制源 snapshot 当时的 data，不能回读 Core 的“当前 data”，否则异步捕获
会混入后续 step。

### 5.3 线程

主要线程关系：

- GUI 模式下 GLFW/MuJoCo RenderLoop 必须运行在主线程。
- Server 仿真循环运行在 worker thread。
- 每个 camera source 和 LiDAR source 有独立 Sensor worker。
- eCAL callback 只做解析、状态转交和 mailbox/queue 写入。
- Adapter 用户控制逻辑始终运行在应用自己的业务线程。

所有 teardown 都遵循：停止接收新 callback -> 排空 callback lease -> 停止 Sensor worker ->
销毁 endpoint -> 最后考虑 `eCAL::Finalize()`。禁止持有 Core/Server 主锁等待线程 join 或日志
sink I/O。

## 6. 完整仿真循环

### 6.1 启动

1. 生成式配置解析完整命令行，加载 YAML，再应用 CLI override；任何失败都不修改调用方配置。
2. Core 原子加载 MJCF；配置了 `initial_keyframe` 时应用 keyframe、清零 ctrl 并调用
   `mj_forward()`。
3. Core 建立具名 Sensor catalog 和 Actuator catalog。
4. Server 校验 camera/LiDAR profile，建立启动期固定的 Stream catalog 和 snapshot pool。
5. eCAL service、publisher 和 subscriber 全部创建成功后，模型立即进入 `RUNNING`。
6. GUI 模式启动主线程 viewer；headless 模式只运行仿真线程。离屏 Sensor renderer 按需在
   自己的 worker thread 中创建。

启动失败必须事务性回滚。publisher、service 或 endpoint 创建失败属于启动失败；不能留下
部分可见的 Server。

### 6.2 每个 Tick 的控制路径

1. `OpenTick()` 固定本 Tick 的 required Controller 集合和 recipient 集合。
2. Core 在 step 前读取当前具名 MuJoCo Sensor，按每个 Controller 的 registration layout
   生成 immutable Tick。
3. `ACTIVE` Controller 是 Barrier required member；`TIMED_OUT` Controller 只接收恢复 Tick，
   不阻塞 Barrier。
4. Server 持续尝试发布 Tick。重发同一 Tick 时 payload 必须完全不变。
5. SDK callback 校验 identity 后发送 Heartbeat，并把 Tick 放入有界 mailbox。
6. 业务线程调用 `WaitForTick()` 得到唯一 outstanding Tick，运行控制算法，然后显式
   `Commit()`。
7. SDK 在本地校验 command 名称、唯一性、标量维度、finite 值和 ctrlrange，并按注册 layout
   规范化 wire payload。
8. Server 校验 session/epoch/tick/ownership/layout。所有 required Commit 到齐后统一写 ctrl，
   只调用一次 `mj_step()`。
9. step 完成后依次进入 Observer completed-step 路径和 SensorRuntime completed-step 路径。
10. ServerStatus、日志指标和 GUI 快照按各自周期更新，不参与 Barrier。

没有 Controller 时 required set 为空，Server 仍会自动 step。为避免 CPU 空转，下一步从上次
step 完成的 wall time 起等待一个模型 timestep；延迟时不连续补步追赶。

### 6.3 超时、恢复和局部错误

Controller 超时时：

1. 状态转为 `TIMED_OUT`。
2. 它拥有的 actuator 立即清零。
3. 它从当前 Barrier 移除，其他 Controller 和 Server 继续运行。
4. Server 仍向它发送 Tick，但不等待回复。
5. 合法恢复 Commit 只证明连接恢复，不应用其中的 command。
6. 下一 Tick 才重新转为 `ACTIVE` 并加入 Barrier。

同名新注册是否能替换 `TIMED_OUT` session 由 Server 参数控制。允许时，新 session 顶替旧
session；旧 SDK 从 ServerStatus 发现 session 被替换后终止。运行期 Unregister 同样只清零并
释放该 Adapter 资源。

非法 command 或 `COMMIT_ERROR` 只把来源 Controller 置为 `ERROR` 并移出 Barrier。单个
Adapter 的错误不能把 Server 推入全局 `ERROR_PAUSED`，也不能令进程退出。identity、version
或 tick 不匹配只拒绝消息，不破坏当前合法 session。

### 6.4 completed-step 统一出口

所有真正执行过 `mj_step()` 的路径都必须进入同一个 post-step 流程，包括：

- 正常 Barrier 完成。
- Controller timeout 后 Barrier 被释放。
- Unregister 或局部 command error 导致 Barrier 完成。
- 没有 Controller 的空 Barrier 自动 step。

该统一出口先收集 Observer frame，再调度 Sensor capture。新增任何 step 路径时，必须证明它
不会跳过 Observer/Stream，否则同步小数据和异步传感器会出现静默缺帧。

## 7. 三类 Adapter 生命周期

### 7.1 Controller: `SyncAdapterClient`

正常生命周期：

```text
constructed
  -> Start/Register (PROVISIONAL)
  -> create endpoints
  -> Activate (ACTIVE)
  -> WaitForTick
  -> user computes command
  -> Commit
  -> repeat
  -> Unregister
  -> stopped
```

约束：

- 同时最多有一个 outstanding Tick。
- outstanding 未 Commit 前再次 Wait 会失败。
- 相同 Tick 不重复交付给业务逻辑。
- 已提交 Tick 被 Server 重发时，SDK 自动原样重发缓存 Commit。
- `TIMED_OUT` 期间 mailbox 为单元素 latest-value，避免恢复时积压历史控制 Tick。
- `STOPPED`、匹配 session 的 `ERROR`、session 被替换和本地 `Stop()` 都会唤醒无限 Wait。
- `Commit()` 传输成功只表示本地已缓存并尝试 publish，不是 Server 业务确认。

### 7.2 Observer: `ObserverClient`

生命周期与 Controller 类似，但没有 Commit：

```text
Start/Register -> Activate -> WaitForObservation -> ... -> Unregister
```

Observer 只声明 observation，不允许声明 command，不进入 Controller Barrier。delivery 支持：

- every-N-completed-step。
- simulation-time period。

period 使用 simulation time deadline 锁相推进，不能使用 wall clock，也不能从迟到的实际发送
时刻重新计时。SDK 使用有界 FIFO/drop-oldest queue，并暴露丢帧计数。

### 7.3 Stream: `SensorStreamClient`

Stream consumer 不注册控制资源。它先发现 Server 的静态 Stream catalog，再按 stream name
建立 subscriber。每个 stream 在以下两种消费模式中二选一：

- blocking wait：调用方取得拥有 payload 的 frame。
- borrowed-view callback：只在 callback 生命周期内借用 payload；跨 callback 保存必须显式
  `Copy()`。

SDK 严格校验 epoch、framing version、format、shape、stride、payload 长度和 sequence。
ServerStatus 中对应 stream 的 `STREAM_ERROR` 是粘性 terminal error，会唤醒 Wait；其他 stream
仍可继续。

## 8. Sensor 数据模型

### 8.1 同步小数据

Core 将所有具名且维度固定的 MuJoCo `sensordata` 资源加入 Sensor catalog，包括 joint、IMU、
site force/torque、frame、subtree、energy、contact/tactile、plugin/user sensor 等。descriptor
包含 MuJoCo type、datatype、needstage、interval、phase、delay、object/frame、unit 和插值信息。

Controller Tick 在 step 前读取当前小数据；Observer 在 step 后读取新状态。两者读取同一个
Core 主状态，但只有 Controller 参与 Commit Barrier。

### 8.2 异步大数据

`mujoco_sim_sensor` 当前支持：

| 来源 | 输出 | 坐标/编码 |
| --- | --- | --- |
| fixed perspective camera | `RGB8` | packed RGB |
| fixed perspective camera | `DEPTH32F` | 米制 optical-Z，little-endian float32 |
| fixed perspective camera | `SEGMENTATION32U` | geom id，little-endian uint32 |
| fixed perspective camera | `XYZ32F` | ROS optical organized XYZ |
| named site + `mj_multiRay()` | `XYZ32F` | ROS body organized LiDAR，miss 为 NaN |

raw frame 使用固定 24-byte little-endian 前缀，后接 Protobuf header 和原始 payload。header
上限 1 MiB，payload 上限 512 MiB。

### 8.3 调度与背压

每个 source 的 `rate_hz` 按 simulation time 调度，实际 capture 发生在 completed-step 离散
边界。wall-clock 变慢不会改变 frame timestamp。

支持三类背压：

- `latest`：深度固定为 1，只保留最新 capture。
- `fifo`：有界队列，可 drop-oldest 或 drop-newest。
- `blocking`：队列深度 `1..64`，等待 capture 和 publish 尝试完成，不保证 eCAL `Send()`
  成功或 subscriber 已处理。

无 subscriber 时不 capture。单个输出序列化失败只将该输出置为 `STREAM_ERROR`；camera capture
失败隔离该 camera source 的所有输出；其他 worker 和仿真循环继续。单次 `Send()` false 是
可重试 publish failure，不应被升级为永久 Stream error。

## 9. GUI 与 headless

默认模式是 GUI，使用仓库内 MuJoCo 3.10 `simulate` UI：

```bash
source install/setup.bash
mujoco-sim-server --model path/to/model.xml
```

显式 headless：

```bash
source install/setup.bash
mujoco-sim-server --headless --model path/to/model.xml
```

YAML 中设置 `headless: true` 后，可以用 `--gui` 双向覆盖。`--headless` 与 `--gui` 互斥。

注意两种“无窗口”情况不同：

- 没有配置 camera stream：headless 不创建 viewer，也不需要图形渲染。
- 配置 camera stream：headless 不创建窗口，但 Sensor worker 仍需要 EGL surfaceless、EGL
  pbuffer 或 OSMesa 进行离屏渲染。

MuJoCo OpenGL dispatch 是进程级状态。renderer 首次选择 EGL 或 OSMesa backend family 后，
同进程不能切换。GUI 的 auto 模式只允许 EGL；显式 OSMesa 与 GUI 同进程组合必须拒绝。

Sensor Demo 场景已经移除天花板。模型 contract 测试会检测覆盖 arena 的顶部 box，避免后续
修改意外恢复封闭顶板，同时不会把旋转横梁误判为天花板。

## 10. 配置体系

### 10.1 生成器

`mujoco_sim_config` 提供 `mujoco_sim_generate_config()` CMake API。schema 在构建期生成：

- C++20 配置 DTO。
- CLI11 option 声明和帮助文本。
- yaml-cpp 严格加载代码。
- build tree/install tree consumer 所需源文件和导出信息。

运行时不读取 schema。字段级类型、范围、enum、array、object 和 object sequence 由生成代码
校验；跨字段业务约束由各包转换层校验。

### 10.2 统一语义

所有生成式配置入口遵循：

```text
CLI explicit override > YAML > generated C++ defaults
```

该优先级与 `--config` 在 argv 中的位置无关。解析先写入临时值，只有 YAML、CLI 和最终语义
校验全部成功后才写回调用方配置。

YAML 必须是单个顶层 mapping，并严格拒绝：

- 未知字段和重复字段。
- 错误类型、嵌套结构错误和非法 enum。
- 负数、整数溢出和业务范围错误。
- 必填 path/resource/server name 为空。

YAML 中的相对 path 按 YAML 文件所在目录解析，而不是当前工作目录。

### 10.3 Server 关键参数

- `model_path`：MJCF 路径，必填。
- `server_name`：eCAL 命名空间标识。
- `initial_keyframe`：可选具名 keyframe。
- `headless`：是否创建 viewer。
- `max_ticks`：从 Server 启动后第一步累计的总 step 上限；`0` 表示无限。
- `tick_timeout_ms`：Controller Barrier timeout。
- `activation_timeout_ms`：PROVISIONAL session 激活期限。
- 同名超时替换开关：决定新 session 是否可顶替同名 `TIMED_OUT` Adapter。
- Sensor source/profile、rate、queue 和 backpressure 配置。
- spdlog level、console、轮转文件和指标窗口配置。

`max_ticks` 主要用于自动化测试和有限批处理，不是正常运行必须项。正常 Demo 使用无限 Server，
由 launcher 或操作者显式终止。由于无 Adapter 时 Server 也会 step，有限测试必须把 Adapter
接入前的自主 step 计入预算。

## 11. 日志、状态与错误边界

### 11.1 日志

Server 使用 spdlog：

- 默认写 stderr，帮助文本写 stdout。
- 可选有界轮转文件。
- 生命周期、状态转换、协议拒绝和 timeout 使用适当级别记录。
- 每 Tick/Commit 细节只在 trace 级别记录。
- 高频外部错误使用低基数限流 key。
- 不记录完整协议 payload 或 command 数值。
- sink I/O 不在 Core mutex 内执行。

### 11.2 状态工具

`mujoco-sim-status` 是 Server 包内的独立轻量可执行文件，不链接 Core、MuJoCo、GUI 或 Server
logger。常用命令：

```bash
source install/setup.bash
mujoco-sim-status --server-name default
mujoco-sim-status --server-name default --watch --interval-ms 1000
mujoco-sim-status --server-name default --json
mujoco-sim-status --server-name default --watch --json
```

watch + JSON 输出 NDJSON。有效 ServerStatus 与资源查询错误独立展示，资源查询失败不能覆盖
已经收到的状态。

### 11.3 错误隔离原则

- 单个 Controller timeout/error/unregister：只影响该 Controller。
- Observer timeout：只把对应 Observer session 置为 `TIMED_OUT`，不影响 Barrier。
- 单 Stream 序列化或不可恢复的输出错误：只把对应 Stream 置为 `STREAM_ERROR`。
- camera capture error：只隔离对应 camera source group。
- Sensor publisher 的 `Send()` 返回 `false`：记录 publish failure 并在后续 frame 重试，不置
  永久错误；subscriber count 查询或 `Send()` 抛出异常时，对应 Stream 进入 `STREAM_ERROR`。
- Tick、Observation 和 ServerStatus transport 的瞬时查询/发送异常：限流记录并在后续周期
  重试，不退出 Server。
- callback、render loop、Server thread 和 Sensor worker 顶层都有异常边界，异常不能越过
  eCAL/std::thread 边界触发 `std::terminate()`。
- endpoint teardown 中某一清理动作失败时继续 best-effort 清理其余对象；若残留 endpoint
  仍依赖 eCAL，则跳过 `eCAL::Finalize()`，优先保证对象依赖顺序安全。

## 12. 启动与验证

### 12.1 全仓构建测试

```bash
colcon list --topological-order
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select \
  mujoco_sim_config \
  mujoco_sim_protocol \
  mujoco_sim_core \
  mujoco_sim_sensor \
  mujoco_sim_server \
  mujoco_sim_adapter_sdk \
  mujoco_sim_sample_adapter \
  mujoco_sim_wbr_lqr_demo \
  mujoco_sim_sensor_demo
colcon test-result --verbose
```

最后一次 clean 结果为 `50 tests, 0 errors, 0 failures, 0 skipped`。

### 12.2 Sample Adapter

终端 1：

```bash
source install/setup.bash
mujoco-sim-server \
  --headless \
  --model core/mujoco_sim_core/test/testdata/simple_actuator.xml \
  --max-ticks 0
```

终端 2：

```bash
source install/setup.bash
mujoco-sim-sample-adapter --ticks 5 --command-value 0.1
```

Adapter 完成后显式注销；无限 Server 由操作者终止。

### 12.3 WBR LQR Demo

```bash
source install/setup.bash
mujoco-sim-server \
  --config install/mujoco_sim_wbr_lqr_demo/share/mujoco_sim_wbr_lqr_demo/config/wbr_server.yaml \
  --headless
```

另一个终端：

```bash
source install/setup.bash
mujoco-sim-wbr-lqr-adapter \
  --config install/mujoco_sim_wbr_lqr_demo/share/mujoco_sim_wbr_lqr_demo/config/wbr_lqr.yaml
```

使用 `--gui` 可运行同一控制闭环的图形模式。控制逻辑保留上游 IMU fusion、FSM、VMC/LQR 和
六电机力矩路径；资源名由 YAML semantic slot 映射，不应在控制公式中写死模型名称。

### 12.4 Sensor/Mapping Demo

```bash
source install/setup.bash
mujoco-sim-sensor-demo-launch \
  --server-config demos/mujoco_sim_sensor_demo/config/sensor_demo_server.yaml \
  --demo-config demos/mujoco_sim_sensor_demo/config/sensor_demo.yaml \
  --output-dir /tmp/mujoco_sensor_demo \
  --mode headless
```

该 launcher 编排以下进程：

1. Server。
2. Controller：同步轮状态/IMU并提交四轮 command。
3. State observer：验证非 Barrier 小数据。
4. Camera consumer：验证 RGB/depth/segmentation/XYZ 并输出 PNG。
5. Mapper：组合 Observer IMU 与异步 LiDAR，使用 PCL ICP 建图。
6. 测试模式 truth recorder：仅在运行结束后评分，不向算法提供真值。

输出包括 PCD、trajectory CSV 和 top-down PNG。launcher 对全部子进程实施有界
`SIGINT -> SIGTERM -> SIGKILL`，确保异常子进程不会令 CI 无限等待。

## 13. 测试与审查基线

最终验证不只包含普通 CTest：

- clean colcon build/test：9 包，50 项测试全通过。
- Protocol、Core、Sensor、Server、Adapter SDK、Sensor Demo 严格告警构建通过：
  `-Wall -Wextra -Wpedantic -Werror`。
- Server ASan/UBSan：`11/11` 通过。
- Adapter SDK ASan/UBSan：基于最新 install 重建后 `7/7` 通过。
- Adapter SDK TSan：59 项纯并发单元测试及 static-lifetime 测试通过。
- Sensor install consumer 在完整 leak detection 下通过。
- Sensor Demo headless 一圈和 GUI Stream E2E 均通过。
- Shell `bash -n`、package manifest XML 和 `git diff --check` 通过。

最终 Sensor Demo clean headless 指标：

| 指标 | 结果 |
| --- | ---: |
| 地图点 | 740378 |
| pose | 287 |
| path length | 29.2406 m |
| closure error | 2.20075 m |
| acceptance | 1.0 |
| ATE | 1.06603 m |
| translation RPE | 0.0244271 m |
| yaw RPE | 0.00244998 rad |
| scale | 0.954829 |
| fitness p95 | 0.00680103 |
| landmark inlier ratio | 0.595982 |
| landmark residual p90 | 0.53403 m |

代码和文档按 Protocol/Core、Sensor/Server、Adapter SDK、Sensor Demo/集成四个面完成多轮
subagent 审查，最终无剩余高、中、低严重级仓库代码问题。

## 14. 已知限制与外部风险

### 14.1 V1 明确不做

- 运行期修改 Stream catalog。
- 动态 contact list。
- 多 Controller 同时控制同一 actuator。
- Controller 优先级或仲裁。
- observation/command 通配符声明。
- 图像压缩、视频编码、GPU 直通。
- ROS 2 runtime 集成。
- `SensorStreamClient` 内建跨 stream 时间同步或 QoS negotiation。

### 14.2 第三方 sanitizer 结果

- 完整 LeakSanitizer 下，系统 `libecal_core.so.6` named-event 在进程退出时残留 123 B；测试
  业务断言通过，关闭 leak detection 后 SDK ASan/UBSan `7/7` 通过。
- 真实 eCAL TSan 会报告其 SHM/service/named-event 内部竞态；仓库纯并发测试和相同路径的
  ASan/UBSan、普通 E2E 通过。没有用 suppression 把第三方报告伪装成仓库通过。
- Mesa EGL/OSMesa 驱动退出时保留进程级全局分配，因此 GL sanitizer 测试使用
  `detect_leaks=0`。

### 14.3 环境依赖

当前本机 `rosdep check` 没有未知 key，但 apt 状态可能报告 `ros-jazzy-ecal`、`xvfb` 和
`binutils-dev` 尚未满足；开发机已有可用系统 eCAL、Xvfb 和工具链。新环境应先核实：

- MuJoCo vendored binary 与目标架构匹配。
- eCAL 6 开发包和运行时配置可用。
- GUI 有 GLFW/OpenGL/libpng；CI GUI 测试有 Xvfb。
- headless camera 有 EGL 或 OSMesa。
- Sensor Demo 有 PCL common/filters/registration。

## 15. 后续开发建议

建议按以下顺序继续，而不是同时扩大协议、调度和算法范围：

1. **运行诊断可观测性**：把 Stream queue depth、capture latency、publish failure、Observer
   drop 和 Barrier latency 做成稳定的状态/指标接口，避免只依赖日志。
2. **Stream catalog 演进协议**：设计 catalog revision、运行期增删和 Client 重订阅状态机；
   必须先解决 epoch/revision 与 borrowed callback 的 teardown 关系。
3. **时间同步 API**：在应用层经验稳定后，再抽象 Observer + 多 Stream 的 simulation-time
   pairing；不要把 wall-clock 同步引入 SDK。
4. **数据传输效率**：评估 eCAL SHM 零拷贝、payload pool、压缩和视频编码。保持 framing
   version 可演进，不破坏现有 raw frame 上限和 endian 契约。
5. **Sensor 范围**：增加 orthographic camera、可配置 LiDAR firing pattern、噪声/延迟模型，
   并继续把数学实现放在 transport-free `mujoco_sim_sensor`。
6. **控制仲裁**：若要支持多个 Controller 共享 actuator，应先定义协议级 ownership、优先级、
   timeout fallback 和确定性合并规则，不能直接放宽当前独占校验。
7. **第三方升级**：独立验证较新 eCAL/Mesa 是否消除现有 LSan/TSan 报告，再决定是否调整
   teardown 或 sanitizer policy。

每项改动仍应遵循 V1 已建立的门禁：生产行为必须有自动化测试，安装态静态库 consumer 必须
验证，真实 eCAL/GUI/Stream 行为需要 E2E，最终差异交由 subagent 复审。

## 16. 接手检查清单

开始开发前：

1. 阅读 `doc/plans/v1/v1.plan` 的包职责、仿真循环和最终验证章节。
2. 运行 `git status --short`，识别并保留本文第 1 节记录的外部未提交改动和运行输出。
3. 运行 `colcon list --topological-order`，确认 9 个包均可发现。
4. clean build/test，确认本机基线不是依赖旧 build/install 产物。
5. 根据改动范围先跑对应包测试，再跑全仓测试。

修改架构时必须持续确认：

- Server 是否仍是唯一主 MuJoCo 状态和 step 所有者。
- 新错误是否被限制在最小 Adapter/Stream/source 范围。
- 新 step 路径是否经过统一 completed-step Observer/Stream 出口。
- callback teardown 是否先阻止新访问并排空 lease。
- GUI/Sensor capture 是否只使用快照且不持主锁执行重工作。
- 配置是否仍由生成器提供且保持事务性和 CLI > YAML > defaults。
- Controller Tick 中既有同步 observation 和 Commit Barrier 语义是否保持兼容。
- 协议新增是否 additive，是否保留既有 field number、enum wire value 和 golden bytes。

完成开发后：

```bash
git diff --check
colcon test-result --verbose
```

并根据风险补跑严格告警、ASan/UBSan、TSan、真实 eCAL、GUI/Xvfb、headless EGL/OSMesa 和安装态
consumer。不要只以单元测试代替跨进程生命周期与 teardown 验证。
