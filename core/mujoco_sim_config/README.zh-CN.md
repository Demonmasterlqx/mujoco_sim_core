# mujoco_sim_config

[English](README.md) | [简体中文](README.zh-CN.md)

用于生成 C++20 配置结构体以及严格 YAML/CLI 解析器的纯 CMake/colcon 包。
运行时链接 `CLI11::CLI11` 和 `yaml-cpp::yaml-cpp`；生成过程使用 Python 3
和 PyYAML。

## CMake API

```cmake
find_package(mujoco_sim_config CONFIG REQUIRED)

mujoco_sim_generate_config(
  TARGET server_config_generated
  SCHEMA config/server_config.schema.yaml
  OUTPUT_NAME server_config  # 可选；默认值为 TARGET
)

target_link_libraries(server PRIVATE server_config_generated)
```

该函数会创建一个库目标，并在使用方的构建树中生成
`server_config.hpp`/`server_config.cpp`。未指定 `OWNER_TARGET` 时，辅助目标会
提供 `MUJOCO_SIM_CONFIG_HEADER` 和 `MUJOCO_SIM_CONFIG_SOURCE` 属性。安装后的
运行时目标为 `mujoco_sim::config`。

对于需要导出的静态库，应将生成的源文件直接附加到该库，避免在其包导出中暴露
生成用的辅助目标：

```cmake
add_library(server ...)
mujoco_sim_generate_config(
  TARGET server_config_generated
  SCHEMA config/server_config.schema.yaml
  OWNER_TARGET server
)
```

指定 `OWNER_TARGET` 后，生成的文件和 `mujoco_sim::config` 都是所属目标的私有
实现。对应的目标属性为 `MUJOCO_SIM_CONFIG_<TARGET>_HEADER` 和
`MUJOCO_SIM_CONFIG_<TARGET>_SOURCE`。可使用 `OUTPUT_DIRECTORY` 覆盖默认生成
目录；`OUTPUT_NAME` 必须是有效的 C++ 标识符。

## Schema

```yaml
version: 1
namespace: mujoco_sim::server::generated
name: ServerConfig
program: mujoco-sim-server
description: Run the simulation server
config:
  flags: [--config, -c]
fields:
  model:
    type: path
    required: true
    non_empty: true
    cli: [--model, -m]
  headless:
    type: bool
    default: false
    cli:
      flags: [--headless]
      inverse_flags: [--gui]
  timeout_ms:
    type: uint32
    default: 1000
    min: 1
    max: 60000
```

支持的字段类型包括 `string`、`path`、`bool`、`uint32`、`uint64`、
`double`、`vector<string>`、`enum`、嵌套 `object`、定长标量 `array` 和
`object_sequence`。支持的约束包括 `required`、`non_empty`、`min`、`max`、
`finite`、`size` 和 `unique`。数组使用 `items` 和 `size`；枚举使用 `values`；
对象使用 `fields`；对象序列使用值为对象的 `items`。每个字段只有显式配置 `cli`
后才会提供 CLI 参数。`inverse_flags` 仅适用于布尔字段。

`required` 表示字段必须通过 YAML 或 CLI 显式提供。必填字段不能同时拥有默认值。
`object_sequence` 元素内的字段不支持 CLI 参数，这是有意的限制，因为 CLI 无法
无歧义地表达元素索引。

## 生成的 C++ API

对于上述 schema，生成器会提供：

```cpp
#include "server_config.hpp"

mujoco_sim::server::generated::ServerConfig config;
std::string error;
if (!mujoco_sim::server::generated::Parse(argc, argv, &config, &error)) {
  // Parse 对错误和 --help 都返回 false；--help 对应的帮助文本由 Usage() 返回。
}
mujoco_sim::server::generated::Load("server.yaml", &config, &error);
const std::string help = mujoco_sim::server::generated::Usage();
```

现有业务 API 可以通过轻量转发封装保持稳定：

```cpp
namespace application {
using ServerConfig = mujoco_sim::server::generated::ServerConfig;

bool ParseCommandLine(int argc, char** argv, ServerConfig* value,
                      std::string* error) {
  return mujoco_sim::server::generated::Parse(argc, argv, value, error);
}
}  // namespace application
```

解析过程具有事务性。YAML 先应用到配置的临时副本，随后应用 CLI 覆盖项并进行
校验，全部成功后才提交结果。因此，无论 `--config` 参数位于什么位置，优先级
始终为 `CLI > YAML > 结构体默认值`。YAML 只接受一个以映射为根节点的文档，并
拒绝未知键、重复键、标量类型强制转换、无符号整数溢出和违反约束的值。YAML
节点位置信息可用时，诊断会包含文件名以及从 1 开始计算的行号和列号。YAML 中的
相对 `path` 值会以配置文件所在目录为基准进行解析；CLI 提供的相对路径仍以进程
工作目录为基准。

## 构建与测试

```bash
cmake -S core/mujoco_sim_config -B build/mujoco_sim_config \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_config -j
ctest --test-dir build/mujoco_sim_config --output-on-failure
```

测试覆盖 schema 生成、生成后配置运行时的行为，以及一个查找已安装包并在其
构建树中生成自身源文件的使用方。
