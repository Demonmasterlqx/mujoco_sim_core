# MuJoCo Simulator Sync Controller V1 注册与通信协议

> 状态：V1 设计基线  
> 范围：MuJoCo Server 与多个同步控制 Adapter 之间的注册、资源声明、Tick/Commit 和失效处理  
> 通信：eCAL Service + eCAL Pub/Sub  
> 资源模型：MuJoCo 原生 Sensor 名称与 Actuator 名称

---

## 1. 文档目标

本文定义 MuJoCo 仿真 Server 与 Sync Controller Adapter 之间的 V1 通信协议。

V1 只解决以下问题：

1. Adapter 如何注册到指定 Server；
2. Adapter 如何声明需要读取的 MuJoCo Sensor；
3. Adapter 如何声明需要控制的 MuJoCo Actuator；
4. Server 如何检查 Actuator 的独占性；
5. Server 如何描述不同 Sensor 的反馈维度；
6. Adapter 如何在每个 Tick 中读取 Observation 并提交 Command；
7. Server 或 Adapter 重启、掉线和超时时如何处理。

V1 不试图构造通用机器人资源系统，也不额外定义 Joint、Body、Site 等抽象接口。

核心原则：

> MuJoCo 中怎样命名和使用 Sensor、Actuator，本协议就怎样命名和使用它们。

因此：

- Observation Resource 的名称就是 MJCF Sensor 的 `name`；
- Command Resource 的名称就是 MJCF Actuator 的 `name`；
- Sensor 的维度由 MuJoCo 编译后的 `sensor_dim` 决定；
- Actuator Command 的维度固定为 1，对应 `mjData.ctrl[actuator_id]`。

---

## 2. V1 范围与非目标

### 2.1 V1 支持

- 一个 MuJoCo Server；
- 多个 Sync Controller Adapter；
- 每个 Controller 每个 Tick 都参与 Barrier；
- Controller 独占声明一组 Actuator；
- 多个 Controller 可同时声明同一个 Sensor；
- 固定长度数值 Observation；
- 固定长度数值 Command；
- Register + Activate 两阶段注册；
- Tick 和 Commit 的幂等重传；
- Server fail-stop；
- Adapter 心跳与 Session 失效。

### 2.2 V1 不支持

- Observer-only Adapter；
- 异步相机和点云；
- 动态长度 Contact List；
- 一个 Actuator 被多个 Controller 叠加控制；
- 优先级覆盖控制；
- 运行中动态增加 Controller；
- 运行中修改 Sensor/Actuator 声明；
- 通配符资源声明；
- 自定义 `joint/foo/position` 等虚拟资源命名；
- 自动把 `qpos`、`qvel` 暴露为 Observation；
- Adapter 直接访问 `mjModel` 或 `mjData`。

如果 Controller 需要关节位置、关节速度、执行器力或 IMU 等数据，应当在 MJCF 中显式创建相应 Sensor 并为其命名。

例如：

```xml
<sensor>
  <jointpos name="left_hip_pos" joint="left_hip_joint"/>
  <jointvel name="left_hip_vel" joint="left_hip_joint"/>
  <actuatorfrc name="left_hip_force" actuator="left_hip_motor"/>
  <gyro name="base_gyro" site="imu_site"/>
  <accelerometer name="base_acc" site="imu_site"/>
</sensor>
```

Adapter 申请的 Observation 名称就是：

```text
left_hip_pos
left_hip_vel
left_hip_force
base_gyro
base_acc
```

---

## 3. 总体架构

```text
┌──────────────────────────────────────────────────────┐
│                    MuJoCo Server                     │
│                                                      │
│  mjModel / mjData                                    │
│  Sensor Catalog                                      │
│  Actuator Catalog                                    │
│  Session Registry                                    │
│  Actuator Ownership Table                            │
│  Tick Barrier                                        │
│                                                      │
│  唯一读取 sensordata 并打包 Observation 的组件       │
│  唯一写入 data->ctrl 并调用 mj_step 的组件            │
└───────────────────────┬──────────────────────────────┘
                        │
                 Tick(controller N)
                        │
       ┌────────────────┼────────────────┐
       ▼                ▼                ▼
┌────────────┐   ┌────────────┐   ┌────────────┐
│ ROS2      │   │ RL Policy  │   │ Native C++ │
│ Adapter   │   │ Adapter    │   │ Adapter    │
└─────┬──────┘   └─────┬──────┘   └─────┬──────┘
      │ Commit          │ Commit          │ Commit
      └─────────────────┴─────────────────┘
                        │
                        ▼
              Server 收齐所有 Commit
                        │
              写入对应 data->ctrl[id]
                        │
                     mj_step()
                        │
                   下一次 Tick
```

Server 是唯一 Coordinator：

- 决定哪些 Controller 是 ACTIVE；
- 决定每个 Controller 拥有哪些 Actuator；
- 维护当前 Tick 的 Barrier；
- 校验和应用所有控制输入；
- 推进物理模拟；
- 产生下一帧 Observation。

---

## 4. MuJoCo 资源映射

## 4.1 Observation = MuJoCo Sensor

V1 中，每个可申请 Observation 必须对应一个具名 MuJoCo Sensor。

资源名称：

```text
MJCF 中 Sensor 的 name 属性原样使用
```

例如：

```xml
<sensor>
  <jointpos name="shoulder_pos" joint="shoulder_joint"/>
  <jointvel name="shoulder_vel" joint="shoulder_joint"/>
  <gyro name="imu_gyro" site="imu_site"/>
</sensor>
```

则可申请的 Observation 名称为：

```text
shoulder_pos
shoulder_vel
imu_gyro
```

Server 使用 MuJoCo API 解析：

```cpp
const int sensor_id = mj_name2id(
    model,
    mjOBJ_SENSOR,
    sensor_name.c_str());
```

不存在时 `mj_name2id` 返回 `-1`。

Sensor 的基本信息来自：

```cpp
const int dimension = model->sensor_dim[sensor_id];
const int address   = model->sensor_adr[sensor_id];
const int type      = model->sensor_type[sensor_id];
const int datatype  = model->sensor_datatype[sensor_id];
```

Sensor 输出位于：

```cpp
data->sensordata[address ... address + dimension)
```

### 4.1.1 Sensor 必须具名

MuJoCo 允许部分对象没有名称，但 V1 的注册接口基于字符串，因此：

> 未命名 Sensor 不进入可声明资源列表。

模型加载后，Server 遍历：

```cpp
for (int sensor_id = 0; sensor_id < model->nsensor; ++sensor_id) {
    const char* name = mj_id2name(model, mjOBJ_SENSOR, sensor_id);
    if (name == nullptr || name[0] == '\0') {
        continue;
    }
    // 添加到 Sensor Catalog
}
```

### 4.1.2 Sensor 数据统一转换为 double

MuJoCo 的 Sensor 数据位于 `mjtNum` 数组中。V1 对外统一使用 `double`：

```text
ObservationValues = repeated double
```

Server 打包时执行数值转换。这样 Adapter 不需要关心 MuJoCo 是以单精度还是双精度编译。

### 4.1.3 Sensor Datatype

Catalog 返回 MuJoCo 原生 `sensor_datatype`，用于辅助 Adapter 解释数据：

- `mjDATATYPE_REAL`：一般实数；
- `mjDATATYPE_POSITIVE`：正数型结果；
- `mjDATATYPE_AXIS`：三维单位向量；
- `mjDATATYPE_QUATERNION`：单位四元数。

V1 不根据 Datatype 拆分字段，也不修改数据顺序。

例如 `dimension = 4` 的 Quaternion 在 Tick 中仍然是连续四个 `double`，顺序完全遵循 MuJoCo 的 Sensor 输出顺序。

### 4.1.4 Sensor 单位

MuJoCo 编译后的通用 Sensor 元数据没有一个适用于所有 Sensor 的统一字符串单位字段。

因此 V1：

- 不在协议中强制提供 `unit`；
- Adapter 根据 Sensor 类型和模型约定解释物理单位；
- 后续版本可以增加可选的模型扩展元数据，但不改变 Sensor 名称。

---

## 4.2 Command = MuJoCo Actuator Control

V1 中，每个可申请 Command 对应一个具名 MuJoCo Actuator。

资源名称：

```text
MJCF 中 Actuator 的 name 属性原样使用
```

例如：

```xml
<actuator>
  <motor name="left_hip_motor" joint="left_hip_joint"/>
  <position name="left_knee_servo" joint="left_knee_joint" kp="100"/>
</actuator>
```

则可申请的 Command 名称为：

```text
left_hip_motor
left_knee_servo
```

Server 使用：

```cpp
const int actuator_id = mj_name2id(
    model,
    mjOBJ_ACTUATOR,
    actuator_name.c_str());
```

每个 Actuator 对应一个标量控制输入：

```cpp
data->ctrl[actuator_id] = command_value;
```

MuJoCo 中 `model->nu` 同时表示 Actuator 数量和 `ctrl` 的维度。因此 V1 中：

```text
每个 Command Resource 的 dimension 固定为 1
```

### 4.2.1 不重新解释 Actuator 语义

V1 不给 Actuator 额外标注：

```text
position command
velocity command
effort command
```

原因是 `data->ctrl[id]` 的物理语义由 MJCF 中该 Actuator 的传动、增益、偏置和激活动力学共同决定。

Adapter 必须根据模型中的 Actuator 定义理解该标量控制值。

也就是说：

```text
left_hip_motor 的 command value 就是 left_hip_motor 的 ctrl
left_knee_servo 的 command value 就是 left_knee_servo 的 ctrl
```

### 4.2.2 Actuator 必须具名

与 Sensor 相同：

> 未命名 Actuator 不进入可声明 Command 列表。

Server 构建 Catalog：

```cpp
for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id) {
    const char* name = mj_id2name(model, mjOBJ_ACTUATOR, actuator_id);
    if (name == nullptr || name[0] == '\0') {
        continue;
    }
    // 添加到 Actuator Catalog
}
```

### 4.2.3 控制范围

Catalog 返回：

```cpp
const bool limited = model->actuator_ctrllimited[actuator_id];
const double min = model->actuator_ctrlrange[2 * actuator_id];
const double max = model->actuator_ctrlrange[2 * actuator_id + 1];
```

Server 在 Commit 阶段校验：

- 值必须为有限数；
- 如果 `ctrllimited=true`，值必须位于 `ctrlrange`；
- V1 默认越界即拒绝当前 Tick 并进入 ERROR_PAUSED；
- V1 不静默 Clamp。

---

## 5. eCAL 接口划分

## 5.1 Service：低频控制面

使用一个 Registry Service：

```text
mujoco_sim.<server_name>.registry
```

逻辑方法：

```text
GetAvailableResources   可选的信息查询
RegisterController      必须
ActivateController      必须
UnregisterController    正常退出时调用
```

`GetAvailableResources` 只是信息获取渠道：

- Adapter 可以调用；
- Adapter 也可以完全不调用；
- 正式配置可以直接写 Sensor/Actuator 字符串；
- Server 不记录 Adapter 是否调用过该方法；
- 注册有效性只在 `RegisterController` 中检查。

## 5.2 Topic：高频数据面

每个 Controller 独立 Tick Topic：

```text
mujoco_sim/<server_name>/tick/<controller_id>
```

共用 Commit Topic：

```text
mujoco_sim/<server_name>/commit
```

共用 Heartbeat Topic：

```text
mujoco_sim/<server_name>/heartbeat
```

Server 状态 Topic：

```text
mujoco_sim/<server_name>/status
```

每个 Controller 使用独立 Tick Topic，是因为不同 Controller 可以申请不同 Sensor 集合，从而拥有不同 Observation Layout。

---

## 6. 标识与生命周期

V1 使用以下标识。

| 字段 | 生成方 | 生命周期 | 作用 |
|---|---|---|---|
| `server_name` | 配置 | 长期 | 定位 Registry Service |
| `server_boot_id` | Server | 每次进程启动 | 区分 Server 重启 |
| `controller_name` | Adapter 配置 | 长期 | 人类可读 Controller 名称 |
| `controller_instance_id` | Adapter | 每次进程启动 | 区分 Adapter 重启 |
| `controller_id` | Server | 当前 Boot | 紧凑运行时编号 |
| `session_id` | Server | 每次注册 | 鉴权当前 Session |
| `simulation_epoch` | Server | 每次仿真 Reset/Reload | 隔离重置前后的消息 |
| `tick_id` | Server | 每个 Epoch 内递增 | 标识控制周期 |

每条 Tick 至少包含：

```text
server_boot_id
simulation_epoch
tick_id
```

每条 Commit 至少包含：

```text
server_boot_id
controller_id
controller_instance_id
session_id
simulation_epoch
tick_id
```

---

## 7. Server 状态机

```text
STARTING
    │ 加载 MJCF、创建 mjModel/mjData、构建 Catalog
    ▼
REGISTRATION
    │ Controller Register + Activate
    ▼
PAUSED
    │ Start 且所有必需 Controller ACTIVE
    ▼
RUNNING
    │ Pause
    ▼
PAUSED

RUNNING
    │ Commit 超时、Controller 掉线、非法命令
    ▼
ERROR_PAUSED
```

规则：

- `REGISTRATION`、`PAUSED`、`ERROR_PAUSED` 允许注册；
- `RUNNING` 拒绝 Register、Activate 和 Unregister；
- 开始运行后冻结 ACTIVE Controller 集合；
- 运行中不改变 Actuator Ownership；
- 任一 ACTIVE Controller 失败，Server fail-stop。

---

## 8. 可选资源查询服务

## 8.1 请求

```protobuf
message GetAvailableResourcesRequest {
  string server_name = 1;
}
```

## 8.2 响应

```protobuf
message GetAvailableResourcesResponse {
  bool success = 1;
  string message = 2;

  bytes server_boot_id = 3;
  uint64 model_revision = 4;

  repeated SensorInfo sensors = 5;
  repeated ActuatorInfo actuators = 6;
}

message SensorInfo {
  string name = 1;
  uint32 dimension = 2;

  // MuJoCo mjtSensor 数值
  int32 sensor_type = 3;

  // MuJoCo mjtDataType 数值
  int32 sensor_datatype = 4;
}

message ActuatorInfo {
  string name = 1;

  bool ctrl_limited = 2;
  double ctrl_min = 3;
  double ctrl_max = 4;
}
```

说明：

- `SensorInfo.name` 是 MuJoCo Sensor 原始名称；
- `SensorInfo.dimension` 等于 `model->sensor_dim[id]`；
- `ActuatorInfo.name` 是 MuJoCo Actuator 原始名称；
- Actuator Command 维度无需返回，V1 固定为 1；
- 列表只包含具名 Sensor 和具名 Actuator。

---

## 9. RegisterController

## 9.1 请求

```protobuf
message RegisterControllerRequest {
  string controller_name = 1;
  bytes controller_instance_id = 2;

  uint32 protocol_major = 3;
  uint32 protocol_minor = 4;

  // MuJoCo Sensor name 数组
  repeated string observations = 5;

  // MuJoCo Actuator name 数组
  repeated string commands = 6;
}
```

示例：

```text
controller_name = "ros2_control"

observations = [
  "left_hip_pos",
  "left_hip_vel",
  "base_gyro",
  "base_acc"
]

commands = [
  "left_hip_motor",
  "left_knee_motor"
]
```

### 9.1.1 请求顺序有意义

V1 保留 Adapter 请求数组的顺序。

Server 按请求顺序构建 Observation Layout 和 Command Layout。

这样 Adapter 可以通过配置明确控制本地数组顺序，但仍必须以 Server 返回的 Layout 为最终依据。

### 9.1.2 重复名称

同一个数组内不允许重复名称。

例如：

```text
commands = ["left_hip_motor", "left_hip_motor"]
```

返回：

```text
DUPLICATE_RESOURCE_NAME
```

### 9.1.3 空数组

- `observations` 允许为空，可支持开环 Controller；
- `commands` 不允许为空，因为 V1 只考虑 Sync Controller。

---

## 10. Server 注册检查

Server 严格按以下顺序检查。

### 10.1 Server 状态

只有以下状态允许：

```text
REGISTRATION
PAUSED
ERROR_PAUSED
```

否则返回：

```text
SERVER_RUNNING
```

### 10.2 协议版本

- `protocol_major` 必须相同；
- `protocol_minor` 使用双方支持的较小值；
- Major 不匹配返回 `INCOMPATIBLE_PROTOCOL_VERSION`。

### 10.3 Controller 名称和实例

- `controller_name` 不能为空；
- `controller_instance_id` 必须存在；
- 同名 Controller 已存在有效 Session 时返回 `CONTROLLER_ALREADY_REGISTERED`。

### 10.4 Observation 存在性

对每个 Observation：

```cpp
const int sensor_id = mj_name2id(
    model,
    mjOBJ_SENSOR,
    observation_name.c_str());

if (sensor_id < 0) {
    return OBSERVATION_NOT_FOUND;
}
```

Observation 不做独占检查。

### 10.5 Command 存在性

对每个 Command：

```cpp
const int actuator_id = mj_name2id(
    model,
    mjOBJ_ACTUATOR,
    command_name.c_str());

if (actuator_id < 0) {
    return COMMAND_NOT_FOUND;
}
```

### 10.6 Command 独占性

Server 维护：

```text
actuator_id -> session_id
```

或者：

```text
actuator_name -> session_id
```

推荐内部使用 `actuator_id`。

如果 Actuator 已被另一个 PROVISIONAL 或 ACTIVE Session 占用，返回：

```text
COMMAND_RESOURCE_CONFLICT
```

错误信息应包含：

```text
actuator name
当前 owner controller_name
当前 owner session_id
```

### 10.7 原子预留

所有 Command 必须一次性检查、一次性预留。

禁止：

1. 先预留前几个 Actuator；
2. 后面的 Actuator 发生冲突；
3. 忘记回滚前面的预留。

正确做法：

```text
先检查全部资源
全部通过后统一写入 Ownership Table
```

Register 成功后立即以 `PROVISIONAL` Session 名义预留 Command，防止两个并发注册请求同时成功。

---

## 11. Sensor 维度与 Observation Layout

## 11.1 Layout 结构

```protobuf
message ObservationLayout {
  uint64 revision = 1;
  uint32 total_dimension = 2;
  repeated ObservationBinding bindings = 3;
}

message ObservationBinding {
  string sensor_name = 1;
  uint32 offset = 2;
  uint32 dimension = 3;

  int32 sensor_type = 4;
  int32 sensor_datatype = 5;
}
```

### 11.1.1 Offset 计算

Server 按 Adapter 请求顺序计算：

```cpp
uint32_t offset = 0;

for (const std::string& name : requested_observations) {
    const int id = mj_name2id(model, mjOBJ_SENSOR, name.c_str());
    const uint32_t dim = model->sensor_dim[id];

    bindings.push_back({
        .sensor_name = name,
        .offset = offset,
        .dimension = dim,
        .sensor_type = model->sensor_type[id],
        .sensor_datatype = model->sensor_datatype[id],
    });

    offset += dim;
}

layout.total_dimension = offset;
```

### 11.1.2 示例

Adapter 请求：

```text
[
  "left_hip_pos",   // dim 1
  "left_hip_vel",   // dim 1
  "base_gyro",      // dim 3
  "base_quat"       // dim 4
]
```

Server 返回：

| Sensor | Offset | Dimension |
|---|---:|---:|
| `left_hip_pos` | 0 | 1 |
| `left_hip_vel` | 1 | 1 |
| `base_gyro` | 2 | 3 |
| `base_quat` | 5 | 4 |

```text
total_dimension = 9
```

Tick 中：

```text
observation_values = [
  hip_pos,
  hip_vel,
  gyro_x, gyro_y, gyro_z,
  quat_w, quat_x, quat_y, quat_z
]
```

具体 Quaternion 分量顺序以 MuJoCo 对该 Sensor 的定义为准。协议不重排。

### 11.1.3 读取 Sensor

简单模型可以读取：

```cpp
const int address = model->sensor_adr[sensor_id];
const int dimension = model->sensor_dim[sensor_id];

for (int i = 0; i < dimension; ++i) {
    output[offset + i] =
        static_cast<double>(data->sensordata[address + i]);
}
```

如果模型配置了 Sensor history/delay，并且使用的 MuJoCo 版本提供 `mj_readSensor`，Server 应优先通过该 API 在当前仿真时间读取，以遵守 MuJoCo 的延迟语义。

### 11.1.4 Layout Revision

每个 Session 的 Observation Layout 在注册时生成并冻结。

```text
Register 成功后不可修改
```

Layout 带有 `revision`。Tick 必须携带相同 Revision。Adapter 发现不一致时不得计算控制，应报告协议错误。

---

## 12. Command Layout

因为每个 MuJoCo Actuator 对应一个标量 `ctrl`，Command Layout 很简单。

```protobuf
message CommandLayout {
  uint64 revision = 1;
  uint32 total_dimension = 2;
  repeated CommandBinding bindings = 3;
}

message CommandBinding {
  string actuator_name = 1;
  uint32 offset = 2;

  bool ctrl_limited = 3;
  double ctrl_min = 4;
  double ctrl_max = 5;
}
```

对第 `i` 个申请的 Actuator：

```text
offset = i
dimension = 1
```

因此：

```text
total_dimension = commands.size()
```

示例：

```text
commands = [
  "left_hip_motor",
  "left_knee_motor",
  "left_ankle_motor"
]
```

布局：

| Actuator | Offset |
|---|---:|
| `left_hip_motor` | 0 |
| `left_knee_motor` | 1 |
| `left_ankle_motor` | 2 |

Commit：

```text
command_values = [hip_ctrl, knee_ctrl, ankle_ctrl]
```

Server 应在注册时缓存：

```text
CommandBinding.offset -> actuator_id
```

从而避免每个 Tick 调用 `mj_name2id`。

---

## 13. RegisterController 响应

```protobuf
message RegisterControllerResponse {
  RegistrationResult result = 1;
  string message = 2;

  bytes server_boot_id = 3;

  uint32 controller_id = 4;
  bytes session_id = 5;

  uint32 effective_protocol_major = 6;
  uint32 effective_protocol_minor = 7;

  uint64 simulation_epoch = 8;
  uint64 timestep_ns = 9;

  ObservationLayout observation_layout = 10;
  CommandLayout command_layout = 11;

  string tick_topic = 12;
  string commit_topic = 13;
  string heartbeat_topic = 14;
  string server_status_topic = 15;

  uint64 heartbeat_interval_ms = 16;
  uint64 lease_timeout_ms = 17;
  uint64 tick_timeout_ms = 18;

  bytes contract_hash = 19;
}
```

Register 成功后 Session 状态为：

```text
PROVISIONAL
```

此时：

- Command 已被预留；
- Controller 尚未进入 Barrier；
- Server 尚不会给它发送 Tick；
- Adapter 需要创建运行时通道并调用 Activate。

推荐 Provisional Timeout：

```text
10 秒墙钟时间
```

超时后：

- 删除 Session；
- 释放所有 Actuator Ownership；
- `session_id` 永久失效。

---

## 14. ActivateController

## 14.1 请求

```protobuf
message ActivateControllerRequest {
  bytes server_boot_id = 1;

  uint32 controller_id = 2;
  bytes controller_instance_id = 3;
  bytes session_id = 4;

  bytes accepted_contract_hash = 5;

  bool tick_subscriber_ready = 6;
  bool commit_publisher_ready = 7;
  bool controller_ready = 8;
}
```

## 14.2 Server 校验

- `server_boot_id` 匹配；
- `controller_id` 存在；
- `session_id` 匹配；
- `controller_instance_id` 匹配；
- Session 当前为 `PROVISIONAL`；
- `contract_hash` 匹配；
- 三个 Ready 标记均为 true；
- Server 不处于 RUNNING。

## 14.3 响应

```protobuf
message ActivateControllerResponse {
  ActivationResult result = 1;
  string message = 2;

  uint64 simulation_epoch = 3;
  uint64 next_tick_id = 4;
  uint64 barrier_revision = 5;
}
```

成功后：

```text
PROVISIONAL -> ACTIVE
```

只有 ACTIVE Session：

- 进入 Tick Barrier；
- 接收 Tick；
- 有权提交 Commit；
- 持有 Actuator 的正式 Ownership。

---

## 15. 为什么必须两阶段注册

如果 Register 成功后 Server 立即把 Controller 加入 Barrier，可能发生：

```text
RegisterResponse 返回
    ↓
Server 立即发布 Tick 0
    ↓
Adapter 尚未创建 Tick Subscriber
    ↓
Tick 0 丢失
    ↓
Server 永久等待 Commit 0
```

两阶段注册解决：

```text
Register：分配 Session、Layout 和资源
Activate：Adapter 明确确认通道与控制器已经就绪
```

这是 V1 保留的必要复杂度。

---

## 16. Start 条件与 Barrier 冻结

Server 开始 RUNNING 前：

1. 确认所有配置为必需的 Controller 都为 ACTIVE；
2. 冻结 ACTIVE Controller 集合；
3. 冻结每个 Session 的 Observation Layout；
4. 冻结每个 Session 的 Command Layout；
5. 冻结 Actuator Ownership Table；
6. 增加 `barrier_revision`；
7. 从 `tick_id = 0` 或当前暂停点开始运行。

运行期间：

```text
不允许 Register
不允许 Activate
不允许 Unregister
不允许修改 Sensor 声明
不允许修改 Actuator 声明
```

---

## 17. Tick 消息

每个 Controller 使用独立 Tick Topic。

```protobuf
message ControlTick {
  bytes server_boot_id = 1;

  uint64 simulation_epoch = 2;
  uint64 tick_id = 3;

  uint64 sim_time_ns = 4;
  uint64 timestep_ns = 5;

  uint64 barrier_revision = 6;
  uint64 observation_layout_revision = 7;

  repeated double observation_values = 8;
}
```

要求：

```text
observation_values.size == observation_layout.total_dimension
```

Tick 的 `sim_time_ns` 表示这组 Sensor 结果对应的 MuJoCo 仿真时刻，不是 eCAL 发送时刻或 Adapter 接收时刻。

---

## 18. Commit 消息

```protobuf
message ControllerCommit {
  bytes server_boot_id = 1;

  uint32 controller_id = 2;
  bytes controller_instance_id = 3;
  bytes session_id = 4;

  uint64 simulation_epoch = 5;
  uint64 tick_id = 6;

  uint64 barrier_revision = 7;
  uint64 command_layout_revision = 8;

  repeated double command_values = 9;
}
```

要求：

```text
command_values.size == command_layout.total_dimension
```

每个 ACTIVE Controller 每个 Tick 必须提交一次完整 Command Vector。

V1 不支持：

- 只提交部分 Actuator；
- 用 Actuator 名称逐项发送；
- 一个 Tick 内多次修改 Command；
- 缺失值继承上一帧。

---

## 19. Tick/Commit 执行顺序

假设当前状态为 `x_n`、时间为 `t_n`：

```text
1. Server 从 mjData 读取每个 Controller 申请的 Sensor。
2. Server 按各自 Observation Layout 打包 Tick(n)。
3. Server 向每个 ACTIVE Controller 发布 Tick(n)。
4. Controllers 并行计算。
5. 每个 Controller 发布 Commit(n)。
6. Server 收齐当前 Barrier 中全部 Commit(n)。
7. Server 校验所有 Command。
8. Server 将每个值写入其所属 data->ctrl[actuator_id]。
9. Server 调用 mj_step(model, data)。
10. MuJoCo 得到 x_(n+1)、t_(n+1)。
11. Server 发布 Tick(n+1)。
```

Server 是 `mjData.ctrl` 的唯一写入者。

Adapter 永远不直接共享或修改 `mjData`。

---

## 20. Commit 校验

Server 收到 Commit 后按以下顺序校验：

1. `server_boot_id` 是否匹配；
2. `controller_id` 是否存在；
3. `controller_instance_id` 是否匹配；
4. `session_id` 是否匹配；
5. Session 是否为 ACTIVE；
6. `simulation_epoch` 是否为当前 Epoch；
7. `tick_id` 是否为当前正在等待的 Tick；
8. `barrier_revision` 是否匹配；
9. `command_layout_revision` 是否匹配；
10. Command Vector 长度是否正确；
11. 所有值是否为有限数；
12. 所有限制型 Actuator 是否位于 `ctrlrange`；
13. 当前 Session 是否已经提交该 Tick。

处理建议：

| 错误 | 处理 |
|---|---|
| 旧 Server Boot | 丢弃并记录 |
| 旧 Session | 丢弃并记录 |
| 旧 Tick | 丢弃 |
| 未来 Tick | ERROR_PAUSED |
| Layout 不匹配 | ERROR_PAUSED |
| 长度错误 | ERROR_PAUSED |
| NaN / Inf | ERROR_PAUSED |
| 超出 ctrlrange | ERROR_PAUSED |
| 同 Tick 不同内容的重复 Commit | ERROR_PAUSED |

---

## 21. eCAL Best-effort 与幂等重传

eCAL Pub/Sub 是 best-effort，Tick 或 Commit 可能丢失，因此 V1 实现应用层重传。

### 21.1 Server 重传 Tick

Server 发布 Tick(n) 后，若未收齐 Commit：

```text
每隔 tick_resend_interval 重发相同 Tick(n)
```

推荐：

```text
tick_resend_interval = 2~5 ms
```

直到：

- 收齐全部 Commit；
- Tick Timeout；
- Pause；
- Controller Session 失效。

### 21.2 Adapter 重传 Commit

Adapter 第一次收到 Tick(n)：

```text
计算 Command
缓存完整 Commit(n)
发布 Commit(n)
```

再次收到相同 Tick(n)：

```text
不重新计算
直接重发缓存的 Commit(n)
```

收到 Tick(n+1)：

```text
视为 Commit(n) 已被接受
清除 Commit(n) 缓存
处理下一 Tick
```

### 21.3 重复 Commit

Server 保存已接收 Commit 的 Payload Hash。

相同 Session、相同 Tick：

- Payload 完全相同：作为合法重传忽略；
- Payload 不同：视为非确定性协议错误，进入 ERROR_PAUSED。

---

## 22. Heartbeat 与 Lease

```protobuf
message ControllerHeartbeat {
  bytes server_boot_id = 1;

  uint32 controller_id = 2;
  bytes controller_instance_id = 3;
  bytes session_id = 4;

  uint64 sequence = 5;
  uint64 last_received_tick = 6;
  uint64 last_committed_tick = 7;

  ControllerRuntimeState state = 8;
  string error_message = 9;
}
```

推荐默认值：

```text
heartbeat_interval = 500 ms
lease_timeout = 2 s
```

超时使用：

```cpp
std::chrono::steady_clock
```

不能使用 MuJoCo 时间或 ROS `/clock`，因为仿真暂停时仿真时间不会前进。

运行中任一 ACTIVE Controller Lease 过期：

```text
不执行当前 mj_step
丢弃当前 Tick 已收集的全部 Commit
进入 ERROR_PAUSED
```

V1 不自动移除掉线 Controller 后继续模拟。

---

## 23. UnregisterController

```protobuf
message UnregisterControllerRequest {
  bytes server_boot_id = 1;

  uint32 controller_id = 2;
  bytes controller_instance_id = 3;
  bytes session_id = 4;
}
```

规则：

- `REGISTRATION`、`PAUSED`、`ERROR_PAUSED`：允许；
- `RUNNING`：返回 `PAUSE_REQUIRED`。

注销后：

1. Session 变为 REMOVED；
2. 释放该 Session 的全部 Actuator Ownership；
3. 删除其 Observation/Command Layout；
4. 增加 `barrier_revision`；
5. 旧 Session 的 Heartbeat 和 Commit 永久无效。

---

## 24. Server 和 Adapter 重启

## 24.1 Adapter 重启

Adapter 每次启动生成新的：

```text
controller_instance_id
```

新进程不能复用旧 `session_id`。

旧 Session 已失效或 Server 处于 ERROR_PAUSED 时：

1. Server 删除旧 Session；
2. 释放旧 Actuator Ownership；
3. 新 Adapter 重新 Register；
4. Server 分配新 `controller_id/session_id`；
5. 新 Adapter Activate；
6. 人工或上层管理逻辑 Resume。

## 24.2 Server 重启

Server 每次启动生成新的：

```text
server_boot_id
```

Adapter 发现 Boot ID 改变后必须：

- 停止发送旧 Commit；
- 清空 Tick 和 Commit 缓存；
- 清空旧 Layout；
- 清空 Session；
- 重新 Register；
- 重新 Activate。

---

## 25. Protobuf 汇总示例

以下为协议骨架，实际工程可以拆分为 `registry.proto` 和 `runtime.proto`。

```protobuf
syntax = "proto3";

package mujoco_sim.v1;

enum RegistrationResult {
  REGISTRATION_OK = 0;
  SERVER_RUNNING = 1;
  INCOMPATIBLE_PROTOCOL_VERSION = 2;
  INVALID_CONTROLLER_NAME = 3;
  CONTROLLER_ALREADY_REGISTERED = 4;
  DUPLICATE_RESOURCE_NAME = 5;
  OBSERVATION_NOT_FOUND = 6;
  COMMAND_NOT_FOUND = 7;
  COMMAND_RESOURCE_CONFLICT = 8;
  NO_COMMAND_RESOURCE_REQUESTED = 9;
  INTERNAL_ERROR = 10;
}

enum ActivationResult {
  ACTIVATION_OK = 0;
  SESSION_NOT_FOUND = 1;
  INVALID_SESSION = 2;
  INVALID_SESSION_STATE = 3;
  CONTRACT_MISMATCH = 4;
  CONTROLLER_NOT_READY = 5;
  ACTIVATION_SERVER_RUNNING = 6;
}

enum ControllerRuntimeState {
  CONTROLLER_INITIALIZING = 0;
  CONTROLLER_IDLE = 1;
  CONTROLLER_PROCESSING = 2;
  CONTROLLER_WAITING_NEXT_TICK = 3;
  CONTROLLER_ERROR = 4;
}

message SensorInfo {
  string name = 1;
  uint32 dimension = 2;
  int32 sensor_type = 3;
  int32 sensor_datatype = 4;
}

message ActuatorInfo {
  string name = 1;
  bool ctrl_limited = 2;
  double ctrl_min = 3;
  double ctrl_max = 4;
}

message GetAvailableResourcesRequest {
  string server_name = 1;
}

message GetAvailableResourcesResponse {
  bool success = 1;
  string message = 2;
  bytes server_boot_id = 3;
  uint64 model_revision = 4;
  repeated SensorInfo sensors = 5;
  repeated ActuatorInfo actuators = 6;
}

message ObservationBinding {
  string sensor_name = 1;
  uint32 offset = 2;
  uint32 dimension = 3;
  int32 sensor_type = 4;
  int32 sensor_datatype = 5;
}

message ObservationLayout {
  uint64 revision = 1;
  uint32 total_dimension = 2;
  repeated ObservationBinding bindings = 3;
}

message CommandBinding {
  string actuator_name = 1;
  uint32 offset = 2;
  bool ctrl_limited = 3;
  double ctrl_min = 4;
  double ctrl_max = 5;
}

message CommandLayout {
  uint64 revision = 1;
  uint32 total_dimension = 2;
  repeated CommandBinding bindings = 3;
}

message RegisterControllerRequest {
  string controller_name = 1;
  bytes controller_instance_id = 2;
  uint32 protocol_major = 3;
  uint32 protocol_minor = 4;
  repeated string observations = 5;
  repeated string commands = 6;
}

message RegisterControllerResponse {
  RegistrationResult result = 1;
  string message = 2;
  bytes server_boot_id = 3;
  uint32 controller_id = 4;
  bytes session_id = 5;
  uint32 effective_protocol_major = 6;
  uint32 effective_protocol_minor = 7;
  uint64 simulation_epoch = 8;
  uint64 timestep_ns = 9;
  ObservationLayout observation_layout = 10;
  CommandLayout command_layout = 11;
  string tick_topic = 12;
  string commit_topic = 13;
  string heartbeat_topic = 14;
  string server_status_topic = 15;
  uint64 heartbeat_interval_ms = 16;
  uint64 lease_timeout_ms = 17;
  uint64 tick_timeout_ms = 18;
  bytes contract_hash = 19;
}

message ActivateControllerRequest {
  bytes server_boot_id = 1;
  uint32 controller_id = 2;
  bytes controller_instance_id = 3;
  bytes session_id = 4;
  bytes accepted_contract_hash = 5;
  bool tick_subscriber_ready = 6;
  bool commit_publisher_ready = 7;
  bool controller_ready = 8;
}

message ActivateControllerResponse {
  ActivationResult result = 1;
  string message = 2;
  uint64 simulation_epoch = 3;
  uint64 next_tick_id = 4;
  uint64 barrier_revision = 5;
}

message UnregisterControllerRequest {
  bytes server_boot_id = 1;
  uint32 controller_id = 2;
  bytes controller_instance_id = 3;
  bytes session_id = 4;
}

message ControlTick {
  bytes server_boot_id = 1;
  uint64 simulation_epoch = 2;
  uint64 tick_id = 3;
  uint64 sim_time_ns = 4;
  uint64 timestep_ns = 5;
  uint64 barrier_revision = 6;
  uint64 observation_layout_revision = 7;
  repeated double observation_values = 8;
}

message ControllerCommit {
  bytes server_boot_id = 1;
  uint32 controller_id = 2;
  bytes controller_instance_id = 3;
  bytes session_id = 4;
  uint64 simulation_epoch = 5;
  uint64 tick_id = 6;
  uint64 barrier_revision = 7;
  uint64 command_layout_revision = 8;
  repeated double command_values = 9;
}

message ControllerHeartbeat {
  bytes server_boot_id = 1;
  uint32 controller_id = 2;
  bytes controller_instance_id = 3;
  bytes session_id = 4;
  uint64 sequence = 5;
  uint64 last_received_tick = 6;
  uint64 last_committed_tick = 7;
  ControllerRuntimeState state = 8;
  string error_message = 9;
}
```

---

## 26. Server 注册伪代码

```cpp
RegisterControllerResponse Registry::Register(
    const RegisterControllerRequest& req)
{
    if (!RegistrationAllowed(server_state_)) {
        return Reject(SERVER_RUNNING);
    }

    if (!ProtocolCompatible(req)) {
        return Reject(INCOMPATIBLE_PROTOCOL_VERSION);
    }

    if (req.controller_name().empty()) {
        return Reject(INVALID_CONTROLLER_NAME);
    }

    if (sessions_.HasLiveName(req.controller_name())) {
        return Reject(CONTROLLER_ALREADY_REGISTERED);
    }

    if (req.commands().empty()) {
        return Reject(NO_COMMAND_RESOURCE_REQUESTED);
    }

    if (HasDuplicates(req.observations()) ||
        HasDuplicates(req.commands())) {
        return Reject(DUPLICATE_RESOURCE_NAME);
    }

    std::vector<SensorBindingInternal> sensors;
    sensors.reserve(req.observations_size());

    uint32_t observation_offset = 0;

    for (const std::string& name : req.observations()) {
        const int id = mj_name2id(model_, mjOBJ_SENSOR, name.c_str());
        if (id < 0) {
            return Reject(OBSERVATION_NOT_FOUND, name);
        }

        const uint32_t dim = model_->sensor_dim[id];

        sensors.push_back({
            .name = name,
            .sensor_id = id,
            .sensor_address = model_->sensor_adr[id],
            .offset = observation_offset,
            .dimension = dim,
            .sensor_type = model_->sensor_type[id],
            .sensor_datatype = model_->sensor_datatype[id],
        });

        observation_offset += dim;
    }

    std::vector<CommandBindingInternal> commands;
    commands.reserve(req.commands_size());

    for (int offset = 0; offset < req.commands_size(); ++offset) {
        const std::string& name = req.commands(offset);

        const int id = mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str());
        if (id < 0) {
            return Reject(COMMAND_NOT_FOUND, name);
        }

        if (actuator_owners_.Contains(id)) {
            return RejectConflict(name, actuator_owners_.At(id));
        }

        commands.push_back({
            .name = name,
            .actuator_id = id,
            .offset = static_cast<uint32_t>(offset),
            .ctrl_limited = static_cast<bool>(
                model_->actuator_ctrllimited[id]),
            .ctrl_min = model_->actuator_ctrlrange[2 * id],
            .ctrl_max = model_->actuator_ctrlrange[2 * id + 1],
        });
    }

    ControllerSession session = BuildProvisionalSession(
        req,
        sensors,
        commands,
        observation_offset);

    // 全部检查通过后原子预留。
    for (const auto& command : commands) {
        actuator_owners_.Insert(command.actuator_id, session.session_id);
    }

    sessions_.Insert(session);
    return BuildRegisterResponse(session);
}
```

---

## 27. Tick 打包与 Command 应用伪代码

### 27.1 打包 Observation

```cpp
ControlTick BuildTick(const ControllerSession& session)
{
    ControlTick tick;
    FillTickHeader(tick);

    auto* values = tick.mutable_observation_values();
    values->Resize(session.observation_total_dimension, 0.0);

    for (const auto& binding : session.sensors) {
        const mjtNum* source =
            data_->sensordata + binding.sensor_address;

        for (uint32_t i = 0; i < binding.dimension; ++i) {
            (*values)[binding.offset + i] =
                static_cast<double>(source[i]);
        }
    }

    return tick;
}
```

### 27.2 应用 Command

```cpp
void ApplyCommit(
    const ControllerSession& session,
    const ControllerCommit& commit)
{
    for (const auto& binding : session.commands) {
        const double value = commit.command_values(binding.offset);

        if (!std::isfinite(value)) {
            throw ProtocolError("non-finite actuator control");
        }

        if (binding.ctrl_limited &&
            (value < binding.ctrl_min || value > binding.ctrl_max)) {
            throw ProtocolError("actuator control outside ctrlrange");
        }

        data_->ctrl[binding.actuator_id] =
            static_cast<mjtNum>(value);
    }
}
```

---

## 28. Adapter 配置示例

Adapter 无需调用资源查询服务，可以直接配置：

```yaml
server_name: robot_sim
controller_name: ros2_control

observations:
  - left_hip_pos
  - left_hip_vel
  - left_knee_pos
  - left_knee_vel
  - base_gyro
  - base_acc

commands:
  - left_hip_motor
  - left_knee_motor
```

启动时：

1. 读取 YAML；
2. 生成 `controller_instance_id`；
3. 直接调用 `RegisterController`；
4. 若资源名错误，Server 返回具体名称；
5. 保存返回 Layout；
6. 创建 Topic；
7. Activate；
8. 进入 Tick 循环。

可选工具可以调用 `GetAvailableResources` 并生成模板 YAML，但这不属于 Adapter 启动的必需流程。

---

## 29. MJCF 示例

```xml
<mujoco model="simple_leg">
  <option timestep="0.001"/>

  <worldbody>
    <body name="base">
      <site name="imu_site"/>

      <body name="thigh">
        <joint name="hip_joint" type="hinge"/>

        <body name="shank">
          <joint name="knee_joint" type="hinge"/>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor
      name="hip_motor"
      joint="hip_joint"
      ctrllimited="true"
      ctrlrange="-20 20"/>

    <motor
      name="knee_motor"
      joint="knee_joint"
      ctrllimited="true"
      ctrlrange="-15 15"/>
  </actuator>

  <sensor>
    <jointpos name="hip_pos" joint="hip_joint"/>
    <jointvel name="hip_vel" joint="hip_joint"/>

    <jointpos name="knee_pos" joint="knee_joint"/>
    <jointvel name="knee_vel" joint="knee_joint"/>

    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_acc" site="imu_site"/>
  </sensor>
</mujoco>
```

资源查询结果应近似为：

```text
Sensors:
  hip_pos    dimension=1
  hip_vel    dimension=1
  knee_pos   dimension=1
  knee_vel   dimension=1
  imu_gyro   dimension=3
  imu_acc    dimension=3

Actuators:
  hip_motor  ctrl_limited=true  ctrlrange=[-20, 20]
  knee_motor ctrl_limited=true  ctrlrange=[-15, 15]
```

Controller 请求：

```text
observations = [
  hip_pos,
  hip_vel,
  knee_pos,
  knee_vel,
  imu_gyro,
  imu_acc
]

commands = [
  hip_motor,
  knee_motor
]
```

Server 生成：

```text
Observation total_dimension = 10
Command total_dimension = 2
```

---

## 30. V1 最终规则

1. Observation 名称严格等于 MuJoCo Sensor `name`。
2. Command 名称严格等于 MuJoCo Actuator `name`。
3. 未命名 Sensor/Actuator 不对外暴露。
4. Adapter 可以不调用资源查询服务。
5. Adapter 注册时直接提交 Sensor 名称数组与 Actuator 名称数组。
6. Server 使用 `mj_name2id` 检查名称并缓存 MuJoCo ID。
7. Observation 可以被多个 Controller 共享。
8. Actuator 必须由一个 Session 独占。
9. Register 成功时即预留 Actuator，Activate 后正式进入 Barrier。
10. Sensor 维度直接使用 `sensor_dim`。
11. Sensor 数据直接使用 `sensor_adr` 指向的连续切片。
12. 所有 Sensor 值对外统一打包为 `double`。
13. 每个 Actuator Command 维度固定为 1。
14. Command 值直接写入 `data->ctrl[actuator_id]`。
15. Actuator 的控制语义完全由 MJCF 定义，协议不重新解释。
16. Runtime 不再传资源字符串，只传固定 Layout 对应的连续数组。
17. 每个 Sync Controller 每个 Tick 必须提交完整 Command Vector。
18. 下一 Tick 是上一 Commit 的隐式 ACK。
19. Tick/Commit 允许幂等重传。
20. 任一 Sync Controller 失败，Server fail-stop。
21. 运行期间不修改 Controller 集合和资源所有权。
22. Server 或 Adapter 重启后必须重新注册和激活。

---

## 31. 官方参考

- MuJoCo API Types：`mjModel.nu`、`sensor_dim`、`sensor_adr`、`actuator_ctrllimited`、`actuator_ctrlrange`、`mjData.ctrl`、`mjData.sensordata`  
  https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html

- MuJoCo API Functions：`mj_name2id`、`mj_id2name`、`mj_readSensor`  
  https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html

- MuJoCo XML Reference：Sensor 与 Actuator 的 MJCF 定义  
  https://mujoco.readthedocs.io/en/stable/XMLreference.html

- eCAL Services：请求—响应服务模型  
  https://eclipse-ecal.github.io/ecal/stable/getting_started/services.html

- eCAL Message Drops：Pub/Sub best-effort 语义  
  https://eclipse-ecal.github.io/ecal/stable/advanced/message_drops.html
