# Adapter SDK 生命周期与包装器收敛实施记录

## 1. 目标与边界

本次工作完成 maintainability plan 阶段 E，并落实 `reduce_wrapper.md` 对 SDK 的要求。范围仅限
`mujoco_sim_adapter_sdk`；Protocol V2 字段、枚举、topic/service 名称以及 Server/Core 行为均未
改变。按本轮决策不保留旧的 SDK 内部测试接缝兼容层。

必须保持的外部语义：

- `Stop()` 只做本地取消，立即禁止新数据、唤醒 Wait，并停止本地 heartbeat；不发远端 RPC。
- `Unregister()` 才清理远端 session。RPC 失败时 session ownership 保留，可在 Stop 或终态错误
  后继续重试。
- 对象是单次生命周期，Stop、成功 Unregister 或 terminal error 后均不能重新 Start。
- Controller 的 `TIMED_OUT` 可恢复；Observer 的 `TIMED_OUT` 是终态。
- 成功 Activate response 与并发 Stop 竞争时，Stop 获胜，预收数据不得泄漏给调用方。

## 2. 已实施结构

三个公开 Client 均改为单 Pimpl facade，公开头文件只保留 options、结果类型、API 契约和
`shared_ptr<Impl>`。选择 `shared_ptr` 是为了支持 `SensorStreamClient` 在当前用户 callback 中
销毁自身：callback 持有 Impl 到返回，teardown 只等待其他线程的 callback。

内部共享组件：

| 组件 | 职责 |
|---|---|
| `AdapterSession` | 唯一本地阶段、远端 session presence 和 `session_id/epoch` owner；拒绝非法或迟到转换 |
| `CallbackGate` | callback lease、禁入、按线程计数和 teardown drain |
| `BlockingMailbox<T>` | 有界 drop-oldest、timeout、Stop、terminal error、单 waiter |
| `AdapterStatusMonitor` | Server 全局终态、session replacement、角色 timeout 策略 |
| `AdapterTransport` | Register/Activate/Unregister/Commit/Heartbeat 的类型化边界 |
| `SensorStreamTransport` | GetSensorStreams discovery 的类型化边界 |

eCAL service discovery、序列化、超时重试和 publisher 串行化只存在于 concrete transport。
Client 测试在对象构造后一次注入 fake transport，随后调用正常生产 API。生产源码已删除
`ActivateWith`、`CommitWith`、`UnregisterWith`、`PublishSerialized` 和重复 `CallService`。

## 3. 状态模型

Controller 与 Observer 共享以下本地阶段：

```text
Constructed -> Starting -> Provisional -> Activating -> Active
     |            |             |             |          |
     +------------+-------------+-------------+----------+-> Stopped
                  +-------------任意运行阶段--------------> TerminalError
```

远端 session presence 不嵌入上述终态：Stop 或 TerminalError 后仍可为 `has_session=true`，直到
Unregister 成功、`NOT_FOUND` 或 `INVALID_SESSION` 才释放。迟到的 Register/Activate response
可以记录远端 session，但不能把 Stopped/TerminalError 对象复活。

Controller/Observer 的数据接收门禁直接由 `AdapterSession::accepting_data()` 判定，不再保存
`accepting_ticks` 或 `accepting_observations`。`monitor_ecal` 是是否观察外部 runtime 退出的策略，
waiter 标记是单等待者并发约束，二者均不表达客户端生命周期。测试只通过合法状态转换构造
session，不再由 `ConfigureForTest` 任意注入不可达组合。

Sensor Stream 不创建 Adapter session，使用独立的 Constructed/Starting/Active/Stopped 阶段。
其跨线程 `stop_requested` 原子量是 transport service 取消信号，不是第二份业务状态机。

## 4. 文件与复杂度

实现按 ownership 不变、职责分文件的方式拆分：

- `*_client.cpp`：构造、选项、生命周期与 transport 建立；
- `*_client_receive.cpp`：协议输入、状态监控、callback/mailbox delivery 与 facade；
- `sync_adapter_client_wait_commit.cpp`：Tick Wait、command 规范化和 Commit 事务；
- `src/internal/adapter_transport.cpp`：eCAL transport。

每个文件仍实现同一个 Client Impl，没有增加 facade-to-wrapper 转发层或第二个状态 owner。当前
SDK production source 在 `lizard -C 15 -L 80` 下 0 告警，所有 `.cpp <= 500` 行，当前最大文件
`sync_adapter_client.cpp` 为 471 行。`tools/quality/check_adapter_sdk_complexity.py` 已把
`CCN <= 15`、函数 `NLOC <= 80` 和 `.cpp <= 500` 自动接入可选 CTest 门禁；检查器自身也有回归
测试。三个公开头的类型、字段、枚举和方法均有 Doxygen 契约，并提供 warning-as-error 文档 target。

## 5. 测试与验收

新增内部原语测试直接覆盖：

- AdapterSession 合法转换、Stop 后迟到响应、terminal error 与远端清理分离；
- CallbackGate 禁止迟到 callback、不等待当前 callback、等待其他线程 callback；
- BlockingMailbox drop-oldest 与 terminal error 优先级。

既有 characterization tests 继续覆盖 Stop/Activate、Stop/Commit/Unregister、失败注销重试、
无限等待唤醒、同 Tick 重传、stream callback 自销毁和延迟 callback。验收命令：

```bash
cmake --build build/mujoco_sim_adapter_sdk -j2
build/mujoco_sim_adapter_sdk/adapter_sdk_test
build/quality-venv/bin/python tools/quality/check_adapter_sdk_complexity.py \
  --lizard build/quality-venv/bin/lizard --root .
```

最终结果为：SDK unit 69/69、带质量门禁的 SDK CTest 9/9、ASAN/UBSAN CTest 7/7、warnings
build 通过、全仓 9 个包构建通过、colcon 汇总 53 项测试零失败。安装产物和静态扫描均通过。
SDK TSAN target 构建成功，但当前容器运行时报 `ThreadSanitizer: unexpected memory mapping`，因此
未把 TSAN 记为通过。当前环境未安装 Doxygen，warning-as-error target 已实现但未在本机执行。
