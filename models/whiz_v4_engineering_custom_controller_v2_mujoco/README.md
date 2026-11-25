# WHIZ V4 机械臂 MuJoCo 模型稳定性说明

本文档说明 `whiz_v4_engineering_custom_controller_v2.xml` 曾经不稳定的原因、诊断证据、
修复方法、控制接口变化和验证流程。本文描述当前运行模型；历史 URDF 仅用于
`robot_state_publisher` 和 ros2_control 资源描述，MuJoCo Server 的物理模型以 MJCF 为准。

## 问题现象与根因

原模型在 `home` 位姿就会快速抖动或被弹开。问题不是单一的求解器参数，而是以下因素叠加：

1. 所有 CAD STL 同时承担视觉与碰撞用途。MuJoCo 对可运动 mesh 使用凸碰撞表示，CAD
   外壳上的孔、凹槽和相邻装配间隙会被凸包填满。
2. `home` 位姿中，MuJoCo 报告 `j1_link` 与 `j3_link` 穿透约 `7.75 mm`，法向力约
   `12.96 N`。对原始三角面独立检查时，两者没有相交，最近的顶点到对方表面距离约
   `8.62 mm`。这是碰撞表示造成的假接触。
3. 启用原始 `base_link` mesh 后，安装在底座内部的 `j0_link` 会产生约 `45.77 mm`
   的真实装配重叠；`base_link` 与 `j1_link` 还会产生约 `18.15 mm` 的凸包假穿透。
4. 原关节的 `damping` 与 `armature` 都为 0。接触冲击作用在很小的转动惯量上，几乎
   没有被动耗散。
5. 原执行器是力矩 motor，而启动配置只加载 `joint_state_broadcaster`。Server 在模型
   加载后将控制量置零，因此 ROS 力矩源接管前机械臂会自由下落。该运动本身符合物理，
   但它会把错误碰撞代理产生的冲击放大成明显抖动。
6. 原关节范围接近一整圈或两整圈，允许大量明显的自碰撞组合。5000 个确定性随机样本
   中有 3706 个样本发生穿透，最深约 `84.82 mm`。
7. 第一轮诊断若只给每个复杂连杆放置一根关节中心线 capsule，会产生另一类漏检：例如
   `j1` 单 capsule 半径为 `26 mm`，但视觉顶点到中心段最远约 `107 mm`；`j4` 半径为
   `24 mm`，最远约 `94.3 mm`。这种代理虽然消除了凸包假接触，却会允许视觉外壳先穿过
   底座或其他连杆。

原始模型在零力矩下运行 5 秒时，2500 个步骤中有 2497 步存在接触，最大关节速度约
`15.14 rad/s`，最大加速度约 `1692 rad/s²`。MuJoCo 没有报告 NaN 或奇异矩阵告警，
说明主要问题是错误接触叠加零力矩自由运动，而不是数值状态已经损坏。

## 修复后的模型结构

### 视觉与碰撞分离

所有 STL 现在只属于 `visual` class：

- `contype="0"`、`conaffinity="0"`，不参与物理接触；
- `group="2"`，保留完整 CAD 外观；
- 不再通过删除 `base_link` geom 来绕过碰撞，因此 GUI 中仍能看到完整底座。

物理碰撞使用单独的 `collision` class：

- 固定底座由前部低矮主体、后部左侧和后部地板三个 box，以及四根上部斜梁 capsule
  组成，在 `j0` 安装柱周围保留实际开口；
- `j0` 使用与圆形转台一致的 cylinder；
- `j1` 到 `j6` 分别使用 3、4、3、4、4、2 个 capsule/sphere 组合，分别覆盖关节壳体、
  偏置连杆、侧向凸起和末端横向宽度；
- `group="3"`，方便在 MuJoCo GUI 中单独显示和检查；
- 使用有界的接触阻抗参数，避免把接触调得比模型时间步所需的更硬。

这些代理不是 STL 的缩放副本。它们表达连杆占据的主要体积，同时保留原网格中的装配
间隙，不会把凹槽和关节开口填成实心凸包。

### 安装开口与碰撞过滤

MJCF 当前没有任何 `<exclude>`，编译后的 `nexclude` 为 0。不能用
`base_link`–`j1_link` 的 body-pair exclude 表达“只排除安装区”，因为 MuJoCo 会对这两个
body 上的所有 geom 全局禁用接触，j1 远端穿入底座也会被静默忽略。

CAD 中 `base_link` 与 `j0_link` 有 1332 个三角面交点，这是安装柱嵌入外壳的设计结果。
当前底座多 box 代理在安装柱周围留出开口，因此 `home` 和安全范围扫掠不需要全局排除。
自动化测试另外使用一个故意越过指令限位的合成姿态，把 `j1_link_collision_distal` 指向
底座，要求得到负距离接触。该姿态不是可下发目标，只用于证明安装开口没有被错误地实现
成全局碰撞屏蔽。

视觉网格采样覆盖测试要求底座和每个运动 link 至少 90% 的采样顶点距 collision 联合体
不超过 `10 mm`，至少 95% 不超过 `20 mm`，最大遗漏不超过 `45 mm`。底座增加斜梁后，
最大遗漏由约 `87 mm` 降到约 `37.4 mm`。阈值用于防止代理退化回单根细 capsule；它
不是 collision 必须包住全部装饰细节的要求。

### 仿真安全限位

下表是当前仿真安全范围，单位为 rad：

| Joint | Lower | Upper | 说明 |
| --- | ---: | ---: | --- |
| `j0` | -0.3 | 3.14159 | 避开底座上部斜梁，保留正向旋转范围 |
| `j1` | -0.1 | 1.7 | 避开折回底座区域 |
| `j2` | -0.1 | 3.14159 | 保留原下限并移除重复转角 |
| `j3` | -1.4 | 3.14159 | 避开与近端机构接触 |
| `j4` | -0.5 | 1.7 | CAD 在约 1.8 rad 开始与前级连杆相交，保留 0.1 rad 裕量 |
| `j5` | -1.05 | 1.9 | CAD 在约 2.0 rad 开始与前级连杆相交，保留 0.1 rad 裕量 |
| `j6` | -3.14159 | 3.14159 | 规范化为单圈范围 |

这些范围通过“其余关节保持 home、单关节完整扫掠”得到，并在两份 URDF 和 MJCF 中保持
一致。对原始 CAD 三角面复查时，j0 负向旋转约在 `-0.44` 到 `-1.92 rad` 之间会与底座
上部斜梁相交，因此下限取 `-0.3 rad`；`j4=1.8 rad` 已出现 55 个三角面交点，
`j5=2.0 rad` 已出现 19 个，因此先前的 j4/j5 2.4/2.8 rad 上限不能继续使用。当前值
是保守的仿真安全限位，不是经过硬件认证的机械限位。拿到制造商或 CAD 的止挡数据后，
应更新三份文件，并重新运行本页的扫掠和组合碰撞测试。

URDF 同时把每个关节的最大 effort 从无意义的 0 改为 `10 N·m`，最大速度设为
`3 rad/s`。effort 与 MJCF actuator 的力矩上限一致。

### 惯量一致性

Git 基线中的 `j6_link` 已具有完整且相互一致的 CAD 惯量数据，本次没有修改这些字段：

- 质量：`0.00666677140172814 kg`；
- 质心：`[0, 0, 0.00852217820789] m`，微小数值零已保留；
- 完整惯量：使用 `j6` 自身数据，不与 `j5` 混用。

新增回归测试会锁定两份 URDF 与 MJCF 的质量一致性，并检查 MJCF 中的质心和惯量量级，
防止后续只修改某一份模型或误用其他连杆数据。若后续实物确认 `j6` 增加了负载，必须
同时更新质量、质心和六个惯量分量，不能只修改质量。

### 数值稳定性

模型仍使用 `0.002 s` 固定时间步，但积分器改为 `implicitfast`。各关节增加了小的
阻尼和电机反射惯量；近端关节值较大，远端关节值较小。它们用于表达传动系统的被动
耗散和反射惯量，不替代主动控制，也不用于掩盖初始穿透。

### 力矩控制与启动时序

七个执行器保持为直接力矩 motor：`gear="1"`，输入与输出范围均为 `[-10, 10] N·m`。
控制值不经过位置或速度内环，MuJoCo actuator 输入 `ctrl` 与施加到对应单自由度关节的
力矩保持 1:1。模型没有启用 `gravcomp`、关节弹簧或隐藏位置保持，因此零输入时机械臂会
在重力下自由运动；这是本模型明确保留的动力学行为。

ros2_control 使用 `actuator_effort` 映射。`j*_tor` sensor 报告 actuator 实际输出力矩。
`controllers.yaml` 会配置并启动：

- `joint_state_broadcaster`；
- 使用 effort command interface 的
  `effort_controllers/JointGroupEffortController`。

Controller Manager 更新率为 `200 Hz`。控制协议异步传递最新目标，不要求 ROS 控制循环
与 MuJoCo 的 `500 Hz` 物理步进逐周期锁步；降低更新率可保留足够的力矩控制带宽，同时
为硬件写路径保留周期裕量。普通非实时 Linux 环境仍可能在进程调度、命令发布或状态查询时
报告偶发控制周期超时；该告警不改变力矩接口语义，也不能被当成硬实时保证。启动时先把 broadcaster
和力矩控制器加载为 inactive；硬件接口只在通过有限值校验的第一帧到达后发布 `/clock`，
launch 以第一帧时钟为就绪信号，再用一次 strict switch 同时激活两个控制器。这避免
broadcaster 把 ros2_control 在首帧前使用的 NaN 哨兵值发布到 `/joint_states`。硬件接口和
JointGroupEffortController 激活时的初始命令均为 0；就绪门不等待第一条用户力矩，因而不
保证启动保持。首帧等待由 GNU `timeout` 限制为 10 秒；如果 Server 没有发布 `/clock`，
waiter 以非零状态退出，launch 通过现有 `_continue_or_shutdown` 路径停止，而不是永久挂起。

Launch 文件还显式向 Server 传入 `--initial-keyframe home` 和 `--headless`。默认无头运行
由 RViz 提供可视化，也避免显示服务器先退出时 MuJoCo GUI 清理路径中的崩溃；需要调试
碰撞组时，可单独运行 Server 并省略 `--headless`。

力矩命令通过 `/arm_controller/commands` 的 `std_msgs/msg/Float64MultiArray` 输入，七个值
依次对应 `j0` 到 `j6`，单位为 N·m。控制器会保持最后一条有效命令；上层停止输出前应先
发送零力矩或停用控制器。力矩闭环、重力补偿、姿态保持和安全停机均由上层控制器负责，
模型不会把力矩命令解释成 rad 位置目标。

## 自动化验证

`test/model_contract_test.cpp` 直接使用仓库提供的 MuJoCo，检查：

- 模型维度、时间步、积分器、21 个 sensor 和 7 个 actuator；
- 关节范围、正阻尼、正 armature、1:1 torque motor 和 `10 N·m` 输入/输出上限；
- STL visual 不参与碰撞，底座和运动 link 只使用预期数量的 primitive collision；
- 模型不含全局 body-pair exclude，并且 j1 远端撞向底座时会产生负距离接触；
- `j6` 保持基线中已有的质量、质心和惯量一致性，不与 `j5` 混用；
- `home` 位姿无接触；
- 每个关节在配置范围内单独扫掠 101 点无接触；
- 一个组合折叠位姿仍能触发自碰撞；
- 从 `j4=1.8 rad` 的约 `2.29 mm` 浅接触开始，由测试侧显式力矩 PD 加重力前馈控制运行
  1000 步，检查有限状态、warning、速度、穿透与恢复；
- 零力矩从 home 自由运动 5000 步，确认它确实不会被隐藏位置环保持，同时保持数值有限且
  无 MuJoCo warning；回归门限还要求最大速度小于 `10 rad/s`、最大离散加速度小于
  `1000 rad/s²`、最深接触穿透小于 `3 mm`，且末态最大关节速度小于 `0.05 rad/s`；
- 测试侧外部力矩控制器从 home 运行 5000 步（10 秒仿真时间），检查无接触、warning、
  非有限状态或异常漂移。

`test/model_config_test.py` 检查：

- xacro 可以展开；
- 七个关节导出 effort command 与 position/velocity/effort state；
- 两份 URDF 的 effort、velocity、上下限及 `j6` 质量有效，并与 MJCF 一致；
- 二进制 STL 的确定性采样满足 collision 联合体的 `10/20/45 mm` 覆盖门限；
- JointGroupEffortController 关节顺序及接口与 MJCF 一致；
- launch 文件语法有效，使用 `home` keyframe 和无头 Server，并在首个有效硬件状态到达
  后激活 `arm_controller`，首帧等待上限为 10 秒；waiter 的超时退出码 124 会生成
  launch shutdown 事件。

本次修复的有界运行结果为：

- 独立 Server 从 `home` 运行 5000 tick 后以 `max_ticks` 正常停止，步进频率约
  `500 Hz`，退出码为 0；
- MuJoCo 单元测试的零力矩 5000 步自由运动中，最大速度约 `7.31 rad/s`、最大离散
  加速度约 `735 rad/s²`、最深穿透约 `1.10 mm`，末态速度满足上述收敛门限。自由下落后
  会出现 `j6`–底座和 `j5`–`j0` 等浅接触，这是机械臂落下后的真实碰撞响应，不是
  `home` 初始位姿中的凸包假穿透；
- 完整 launch 使用 Git `HEAD` 构建的隔离干净 hardware interface 前缀运行当前最终模型约
  98 秒，当前模型包作为上层覆盖；两个控制器均为 `active`，七个 `j0/effort` 到
  `j6/effort` command interface 均为 `available` 和 `claimed`，没有 position command
  interface。Server 持续报告
  `online_controllers=1`、`offline_controllers=0`、`error_controllers=0`；
- 零命令下机械臂按预期受重力运动并在安全限位附近稳定。一次稳定后抽样位置约为
  `[-0.30002, -0.10079, -0.10181, 0.01398, -0.50260, -0.79097, 0.37490] rad`，
  速度约为 `10^-7 rad/s` 或更小，报告力矩全部为 0；该结果证明模型没有隐藏位置保持，
  不是 home 位姿保持精度指标；
- 向 `/arm_controller/commands` 发送
  `[0.2, 0, 0, 0, 0, 0, 0]` 后，`/joint_states` 的 `j0` effort 观测为 `0.2 N·m`，
  其余六轴为 0，验证 ROS 输入按 1:1 力矩语义到达 MuJoCo；随后已恢复零命令；
- 就绪门生效后没有出现 NaN joint state、`TF_NAN_INPUT`、adapter 错误或 Server 掉线。
  非实时验证环境在状态查询和命令发布期间出现 3 次约 `5.3–5.9 ms` 的 ros2_control
  overrun warning；因此本次结果只证明有界功能和数值稳定性，不构成硬实时保证；
- Xvfb 下向终端进程组发送 SIGINT 后，Controller Manager 先停用控制器并注销 adapter，
  MuJoCo 使用 headless 路径，没有再次出现 GUI/Xvfb 关闭时的
  `double free or corruption`，也没有遗留进程。该次 launch 因 RViz 收到进程组 SIGINT
  后以 1 退出而整体返回 1；这是可视化进程的关闭状态，不是 MuJoCo 数值失败。
  Controller Manager 的 `pal_statistics` 线程在 ROS context 关闭时仍输出两条上游关闭期
  diagnostic。独立 Server 的 `max_ticks` 路径退出码为 0，验证了无信号干预的正常初始化、
  稳定运行和正常退出。

在仓库根目录运行：

```bash
colcon build --packages-up-to whiz_v4_engineering_custom_controller_v2_mujoco \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select whiz_v4_engineering_custom_controller_v2_mujoco \
  --event-handlers console_cohesion+
colcon test-result --all --verbose
```

启动完整 ROS 2 集成：

```bash
source install/setup.bash
ros2 launch whiz_v4_engineering_custom_controller_v2_mujoco \
  launch_hardware_interface.py
```

启动后可向 `/arm_controller/commands` 发送七关节力矩数组，例如发送全零命令：

```bash
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

数组顺序固定为 `j0` 到 `j6`，单位为 N·m，每个值会限幅到 `[-10, 10]`。零力矩不会保持
home；要保持姿态，上层必须计算并持续更新所需的反馈和重力补偿力矩。

## 修改碰撞代理时的检查步骤

1. 先在 GUI 中仅显示 group 2，确认视觉网格坐标正确。
2. 再显示 group 3，确认 box/cylinder/capsule/sphere 联合体覆盖关节壳体和偏置结构，且
   不穿过不相邻 link。
3. 在 `home` 执行 `mj_forward`，要求 `ncon == 0`。
4. 单独扫掠每个关节的完整范围，要求没有接触。
5. 运行视觉采样覆盖测试，避免多 primitive 被简化成不能覆盖 CAD 主体的细代理。
6. 执行 j1 远端撞底座与组合折叠姿态，确认分别存在负距离接触；再从浅接触运行短时
   rollout，检查状态有限、无 warning 且可以回到安全范围。
7. 从 home 以零力矩运行至少 5000 步，检查自由运动、接触、warning、状态有限性和最大速度；
   再由测试侧外部力矩控制器运行相同时长，单独检查姿态保持能力。
8. 如果改动了控制接口、actuator 名称或范围，同时更新 MJCF、两份 URDF、xacro、
   controllers YAML、launch、本文档和自动化测试。

## 已知边界

- primitive collision 是稳定、可解释的近似，不是 CAD 表面的毫米级认证模型。
- 当前安全限位来自仿真扫掠；硬件止挡、线束弯曲半径和厂商允许范围仍需实物资料确认。
- 模型不提供内置位置保持或重力补偿。启动、命令源延迟或命令源停止时，零力矩会让机械臂
  受重力运动；姿态精度和安全停机必须由外部力矩控制器保证。
- 新增末端工具或负载时必须更新 `j6` 及负载的质量、质心、惯量、碰撞代理和 actuator
  力矩裕量，并重新运行全部测试。

## English summary

The instability was caused primarily by convex collision representations of concave CAD meshes,
an initial false `j1`–`j3` penetration, zero passive damping, and a torque-controlled arm receiving
zero torque before ROS controller activation. The fixed model separates visual meshes from box and
cylinder/capsule/sphere collision proxies, uses a segmented base with a physical mount opening and no
global body-pair excludes, protects the existing coherent `j6` inertia with regression checks,
tightens limits where CAD
surfaces actually intersect, and adds damping and armature while preserving bounded direct-torque
motors and an effort JointGroupEffortController after a bounded state-ready wait. Zero torque is not
a hold command and gravity compensation remains an external-controller responsibility. Tests cover
visual-proxy sampling, contact-free home and safe single-joint sweeps, explicit base and combined
self-collision, configuration consistency, and a 10-second bounded stability rollout.
