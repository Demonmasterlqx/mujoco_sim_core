# mujoco_sim_config

[English](README.md) | [简体中文](README.zh-CN.md)

Pure CMake/colcon package for generating C++20 configuration structs and
strict YAML/CLI parsers. The runtime links `CLI11::CLI11` and
`yaml-cpp::yaml-cpp`; generation uses Python 3 and PyYAML.

## CMake API

```cmake
find_package(mujoco_sim_config CONFIG REQUIRED)

mujoco_sim_generate_config(
  TARGET server_config_generated
  SCHEMA config/server_config.schema.yaml
  OUTPUT_NAME server_config  # optional; defaults to TARGET
)

target_link_libraries(server PRIVATE server_config_generated)
```

The function creates a library target and generates
`server_config.hpp`/`server_config.cpp` under the consumer build tree. Without
`OWNER_TARGET`, the helper target exposes `MUJOCO_SIM_CONFIG_HEADER` and
`MUJOCO_SIM_CONFIG_SOURCE` properties. The installed runtime target is
`mujoco_sim::config`.

For an exported static library, attach generated sources directly so it does
not expose a generated helper target in its package export:

```cmake
add_library(server ...)
mujoco_sim_generate_config(
  TARGET server_config_generated
  SCHEMA config/server_config.schema.yaml
  OWNER_TARGET server
)
```

With `OWNER_TARGET`, generated files and `mujoco_sim::config` are private to
the owner. Their target properties are
`MUJOCO_SIM_CONFIG_<TARGET>_HEADER` and
`MUJOCO_SIM_CONFIG_<TARGET>_SOURCE`. `OUTPUT_DIRECTORY` can override the
default generation directory; `OUTPUT_NAME` must be a valid C++ identifier.

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

Supported field types are `string`, `path`, `bool`, `uint32`, `uint64`,
`double`, `vector<string>`, `enum`, nested `object`, fixed scalar `array`, and
`object_sequence`. Constraints are `required`, `non_empty`, `min`, `max`,
`finite`, `size`, and `unique`. Arrays use `items` plus `size`; enums use
`values`; objects use `fields`; object sequences use an object-valued `items`.
CLI flags are opt-in per field. `inverse_flags` is valid only for booleans.

`required` means the field must be explicitly supplied by YAML or CLI. A
required field cannot also have a default. CLI flags within an
`object_sequence` item are intentionally unsupported because no item index can
be expressed unambiguously.

## Generated C++ API

For the schema above, generation provides:

```cpp
#include "server_config.hpp"

mujoco_sim::server::generated::ServerConfig config;
std::string error;
if (!mujoco_sim::server::generated::Parse(argc, argv, &config, &error)) {
  // Parse returns false for both errors and --help; --help returns Usage().
}
mujoco_sim::server::generated::Load("server.yaml", &config, &error);
const std::string help = mujoco_sim::server::generated::Usage();
```

Existing business APIs can remain stable through thin forwarding wrappers:

```cpp
namespace application {
using ServerConfig = mujoco_sim::server::generated::ServerConfig;

bool ParseCommandLine(int argc, char** argv, ServerConfig* value,
                      std::string* error) {
  return mujoco_sim::server::generated::Parse(argc, argv, value, error);
}
}  // namespace application
```

Parsing is transactional. YAML is applied to a temporary copy, then CLI
overrides are applied, validated, and committed, giving
`CLI > YAML > struct defaults` regardless of `--config` argument order.
YAML accepts one document with a mapping root and rejects unknown or duplicate
keys, coercive scalar types, unsigned overflow, and constraint violations.
When a YAML node location is available, diagnostics include the file and
one-based line and column information. Relative `path` values loaded from YAML
are resolved against the configuration file directory. Relative paths supplied
by CLI remain relative to the process working directory.

## Build And Test

```bash
cmake -S core/mujoco_sim_config -B build/mujoco_sim_config \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_sim_config -j
ctest --test-dir build/mujoco_sim_config --output-on-failure
```

Tests cover schema generation, generated runtime behavior, and a consumer that
finds the installed package and generates its own build-tree source files.
