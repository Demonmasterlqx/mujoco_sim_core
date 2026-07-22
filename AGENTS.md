# mujoco_sim_core - C++

## Project Overview

mujoco_sim_core is a MuJoCo-based simulation framework implemented as a
multi-package C++ workspace. colcon provides workspace discovery, topological
builds, and test orchestration; every package is otherwise a pure CMake package.
The production runtime is not a ROS 2 system.

The supported baseline is Ubuntu 24.04 on Linux x86_64, CMake 3.22 or newer,
and C++20. The bundled MuJoCo distribution is Linux x86_64 specific. Builds that
target another platform must use a compatible system MuJoCo installation and
must document and test the alternative configuration.

## Repository Layout and Responsibilities

### Root files

- AGENTS.md defines repository-wide engineering and verification rules.
- Readme.md is the user-facing package index, build guide, and current runtime
  overview. Keep its commands and package list consistent with the workspace.
- .clang-format is the formatting authority for first-party C++ source.
- .github/ contains CI workflows. A local check is not a substitute for keeping
  the corresponding CI workflow valid.

### core/

core/ contains production libraries, reusable runtime components, and installed
framework APIs.

- core/mujoco_sim_config owns the schema-driven C++ configuration generator,
  strict YAML/CLI runtime, generator templates, and CMake integration.
- core/mujoco_sim_protocol owns Protocol V4 Protobuf definitions, dense data
  framing, sensor stream framing, identifier validation, layout hashing, and
  endpoint naming.
- core/mujoco_sim_core owns the transport-independent MuJoCo state, resource
  catalog, resource ownership, adapter sessions, command policy, liveness, and
  completed-step data production.
- core/mujoco_sim_sensor owns transport-independent snapshot-based camera and
  ray-LiDAR capture.
- core/mujoco_sim_server is the eCAL composition root. It owns configuration,
  transport endpoints, the single-writer simulation loop, sensor workers,
  logging, status tooling, and the optional GUI.
- core/mujoco_sim_adapter/mujoco_sim_adapter_sdk owns the installed eCAL client
  SDK for Controllers, Observers, and Sensor Stream consumers.

Only production SDK code belongs under core/mujoco_sim_adapter/. Example and
sample adapters belong under demos/. Do not recreate an example adapter package
under core/ or use a removed package as an architectural precedent.

Packages under core/ must never depend on packages under demos/. Preserve the
transport-independent boundaries of mujoco_sim_core and mujoco_sim_sensor:
neither package may acquire eCAL endpoints, RPC handling, callback threads, or
server process responsibilities.

### demos/

demos/ contains complete example applications and robot-specific integrations.
Demo code may include application-specific controllers, mapping algorithms,
configuration conversion, launch scripts, MJCF assets, and demo-scoped library
targets needed for unit testing or installed consumer examples.

- Demo targets must consume installed public APIs from core packages.
- Demo targets must not include core private headers or rely on undocumented
  symbols.
- Reusable simulator infrastructure and production-critical behavior must move
  to the appropriate core package.
- Robot-specific or scenario-specific logic may remain in a demo even when it
  is organized as a library and has comprehensive tests.
- A demo is not a substitute for automated tests.

### doc/

- doc/architecture documents current ownership, lifecycle, thread, ordering,
  and locking behavior.
- doc/protocol documents the current communication contract.
- doc/build documents supported build variants and environment-specific setup.
- doc/performance stores benchmark methodology and recorded evidence.
- doc/plans and doc/refactor contain historical plans, handoffs, and migration
  evidence. They are not authoritative descriptions of the current runtime.

Do not implement behavior solely because it appears in a historical plan.
Confirm current behavior in code, package README files, doc/architecture, and
doc/protocol. When behavior changes, update the applicable current document in
the same change. Do not rewrite historical plans to make them appear current.

### tools/

tools/ contains repository maintenance and quality tooling. tools/quality owns
the pinned complexity checks, their regression tests and baselines, Doxygen
instructions, sanitizer suppressions, and quality-tool requirements. Changes
to a checker require tests for the checker.

### Generated and local directories

build/, install/, log/, CMakeFiles/, Testing/, doxygen-local/, CMakeCache.txt,
cmake_install.cmake, and compile_commands.json are generated or local outputs.
They are not source directories and must not be committed.

All CMake builds must be out of source and use a directory beneath build/.
Never configure CMake with the repository root as the build directory.

## Source-of-Truth Rules

- A configuration schema is the YAML file passed through the SCHEMA argument of
  mujoco_sim_generate_config. Modify that schema or the config generator; never
  edit its generated C++ output.
- core/mujoco_sim_protocol/proto/mujoco_sim/v4/sync.proto is the source for
  generated Protobuf declarations. Never edit sync.pb.h or sync.pb.cc.
- Generated config and Protobuf files belong in the package build tree. Do not
  copy generated files into source directories to work around a build issue.
- Public CMake target names, installed headers, package config templates, and
  package.xml together define the installed package contract.
- Current runtime semantics are documented by package README files,
  doc/architecture, and doc/protocol. Update documentation whenever code changes
  those semantics.

## Third-Party Code

The following trees are third-party or vendored code:

- core/mujoco_sim_core/mujoco/
- demos/mujoco_sim_wbr_lqr_demo/third_party/

Do not apply project formatting, bilingual documentation requirements,
complexity limits, or unrelated cleanup to third-party code. Do not edit
vendored code to hide a problem that should be fixed at a first-party boundary.

An intentional third-party update must:

- identify the upstream project, version, tag, or commit;
- preserve applicable licenses and notices;
- update the package's provenance or UPSTREAM documentation;
- keep first-party adaptation code outside the vendored tree when practical;
- build and test all consumers of the vendored code;
- review the vendor diff separately from first-party changes.

Generated files and third-party files are exempt from the first-party header
documentation rules below. Handwritten compatibility wrappers around
third-party APIs are first-party code and are not exempt.

## Architectural Invariants

The following are repository invariants, not implementation suggestions.

### Simulation ownership

- SimulationCore exclusively owns the live mjModel and mjData, model epoch,
  resource catalog, resource ownership, adapter records, and simulation state.
- After initialization, SimulationLoop is the only production thread allowed to
  invoke or mutate SimulationCore.
- SimulationCore remains transport-independent. It does not own eCAL
  publishers, subscribers, RPC services, callback threads, wall-clock pacing,
  or a generic server event queue.
- Step() is the normal owner of mj_step(). Registration, command submission,
  liveness checks, and transport callbacks must not step the model.
- Each command actuator has exclusive Controller ownership. Registration fixes
  the session's dense resource layout and layout fingerprint.

### Server threading and event flow

- eCAL callbacks validate and decode external input into owned event values and
  enqueue them for SimulationLoop. They never call SimulationCore directly.
- Sensor workers and the GUI consume owned snapshots. They never borrow the
  live mjModel or mjData from SimulationCore.
- Published and queued values own their data. A callback must not enqueue a
  borrowed view whose lifetime ends when the callback returns.
- Missing Controllers, disconnected clients, absent subscribers, and ordinary
  publish failures do not block simulation stepping.
- A configured blocking Sensor fence is the deliberate exception: it may pause
  the single writer after a completed Step. Changes to that fence require
  explicit latency, shutdown, and failure-path tests.
- Preserve the documented event and completed-step ordering in
  doc/architecture/server_runtime.md. Ordering changes are behavioral changes,
  even when message layouts remain unchanged.

### Locks, callbacks, and shutdown

- No Server mutex may be introduced as a general lock around SimulationCore.
- Keep lock ownership narrow and critical sections small.
- Never hold a project lock while invoking an external callback, publishing
  through a transport, waiting on an RPC promise, joining a thread, or calling
  into SimulationCore, unless an existing documented invariant explicitly
  requires it.
- Stop paths must reject new callbacks, wake blocking waits, stop worker loops,
  drain or join active work, and release transport resources in a documented
  order.
- Callbacks may run on communication threads. Public APIs must document callback
  thread, payload lifetime, reentrancy, destruction, and blocking guarantees.
- Changes to locks, atomics, queues, depth-1 mailboxes, keep-last behavior,
  callback gates, timeout handling, or Start/Stop/Join ordering require
  dedicated concurrency tests.

### Timing and high-frequency behavior

- Simulation time, completed-step tick identifiers, wall-clock deadlines, and
  message sequences are distinct concepts and must not be conflated.
- Liveness and timeout code uses steady-clock time where required by the
  existing API. Tests should inject explicit times rather than depend on sleeps
  whenever practical.
- Do not add filesystem access, process execution, unbounded work, verbose
  logging, avoidable allocation, or uncontrolled blocking to a high-frequency
  simulation or communication path.
- Do not introduce implicit synchronization between the Server and external
  clients. Blocking, timeout, retry, queueing, dropping, keep-last, and offline
  behavior must remain explicit and testable.

## Core Coding Rules

- Use C++20 and preserve the naming and API style of the package being changed.
- Format changed first-party C++ files with the repository .clang-format.
  Do not format generated or third-party files.
- Prefer explicit ownership, composition, and small responsibility-focused
  components over deep inheritance or forwarding-only wrappers.
- Avoid hidden global mutable state. Any shared mutable state must have a clear
  owner and synchronization contract.
- Before implementing a feature, identify expected inputs, outputs, failure
  states, ownership, thread context, and interaction with the simulation tick or
  communication lifecycle.
- Preserve existing public APIs unless the task explicitly authorizes a
  breaking change.
- Do not silently change field meanings, units, dimensions, timestamps,
  ordering, resource naming, ownership, timeout behavior, or error semantics.

### File and function size

- A first-party source or header above approximately 500 lines triggers a
  responsibility review; it is not an automatic requirement to split.
- Do not split cohesive code into empty abstractions merely to satisfy a line
  target.
- Simulation Core and Adapter SDK changes must satisfy their checked-in quality
  gates: cyclomatic complexity no greater than 15, function NLOC no greater
  than 80, and production .cpp files no greater than 800 lines.
- Do not update a complexity baseline merely to make a regression pass.
  Baseline updates require an intentional, reviewed explanation.

## Header Documentation Rules

Every first-party header file must contain detailed bilingual documentation in
both English and Chinese. This remains the target for existing headers as well
as new code. Existing documentation debt does not weaken the rule and will be
handled through dedicated migration work.

Public classes, structures, enumerations, type aliases, constants, and
functions must use Doxygen-compatible documentation. English and Chinese text
must express the same behavior and design intent rather than merely translate
the identifier.

Document, where applicable:

- purpose and responsibility;
- parameters and return values;
- ownership and lifetime;
- thread-safety and permitted calling threads;
- blocking or non-blocking behavior;
- callback execution context and payload lifetime;
- error conditions;
- preconditions and postconditions;
- units, dimensions, valid ranges, and special values;
- interaction with simulation ticks, message sequences, or lifecycle states.

Example:

~~~cpp
/**
 * @brief Commits the latest controller command to the simulation server.
 *
 * The command is stored as the latest available control value. This function
 * does not wait for the simulation server to complete a physics step.
 *
 * 提交控制器最新的控制指令到仿真服务器。
 *
 * 该指令会作为当前最新控制值保存。本函数不会等待仿真服务器完成物理仿真步进。
 *
 * @param command Command data to commit.
 *                需要提交的控制指令数据。
 *
 * @return true if the command was accepted; otherwise false.
 *         如果控制指令被成功接收则返回 true，否则返回 false。
 *
 * @thread_safety This function may be called from the controller thread.
 *                本函数可以由控制器线程调用。
 */
bool CommitCommand(const Command& command);
~~~

Document private implementation declarations when their ownership,
synchronization, lifecycle, state transitions, or invariants are not immediately
obvious. Do not add comments that only restate the code. When changing an API,
update both languages in the same change.

Third-party and generated headers are exempt only as defined in the Generated
and Third-Party sections. Test helper headers and handwritten private headers
remain first-party headers.

## Error Handling

- Do not ignore failures returned by MuJoCo, eCAL, Protobuf, operating-system
  APIs, configuration parsers, graphics APIs, or third-party libraries.
- Error messages must identify the failed operation and affected component and,
  where applicable, the adapter, session, resource, topic, endpoint, stream,
  file, or configuration field.
- Preserve the underlying diagnostic when it is available and safe to expose.
- Validate external identifiers, configuration values, dimensions, sizes,
  numeric finiteness, ranges, epochs, sessions, sequences, and payload lengths
  before use.
- Fail during initialization when required configuration, resources, or
  dependencies are invalid.
- Do not use exceptions for expected communication states such as an offline
  adapter, timeout, missing subscriber, or retryable transport failure.
- Do not allow exceptions to cross C ABI, transport callback, thread-entry, or
  package communication boundaries. Translate them into the existing package
  error mechanism with contextual diagnostics.
- Preserve transactional output behavior where an API promises that output is
  unchanged on failure.

## Communication Contract Changes

Communication changes are compatibility-sensitive. Do not change message
fields, field meanings, dense layouts, framing, topic/service names,
registration behavior, resource naming, units, timing, or ownership without
reviewing and updating every affected layer:

- protocol definitions and codecs;
- Server conversion, validation, and transport code;
- Adapter SDK or other clients;
- current API and protocol documentation;
- serialization, malformed-input, and server/client integration tests.

New or modified fields must have explicit dimensions, units, valid ranges,
default behavior, timestamp meaning, tick meaning, sequence rules, and
ownership semantics. Retry, timeout, disconnect, reconnection, and offline
behavior must be specified and testable.

## Package and Dependency Management

- All system and build dependencies are installed and managed with apt.
- Never run rosdep and never add rosdep as a documented setup step.
- package.xml is colcon package metadata; do not assume its dependency names can
  be resolved through the ROS dependency index.
- Do not introduce ROS-specific assumptions unless the target package is an
  explicitly requested ROS 2 integration package.
- Avoid new third-party dependencies when the standard library or an existing
  project dependency provides a reliable semantic match.

Whenever a dependency is added, removed, made public, or made private, review
and update all applicable locations:

- find_package and target_link_libraries in CMakeLists.txt;
- package.xml build, execution, and test dependency declarations;
- installed package Config.cmake.in find_dependency calls;
- exported target interfaces and include paths;
- package consumer tests;
- user-facing build documentation and apt installation instructions.

Each package must remain independently configurable and buildable against
installed dependencies when BUILD_TESTING=OFF. Tests may add declared test-only
dependencies on other workspace packages.

## CMake and Installed API Rules

- Keep targets explicit and use target_compile_features,
  target_include_directories, target_link_libraries,
  target_compile_definitions, and target_compile_options.
- Use BUILD_INTERFACE and INSTALL_INTERFACE correctly for exported targets.
- Do not expose source-private include directories through installed targets.
- PUBLIC and INTERFACE dependencies are part of the installed contract and must
  be represented in the package config template.
- A change to installed headers, exported targets, public dependencies, package
  configuration, installed assets, or executable names requires the relevant
  installed package consumer test.
- Do not suppress warnings for first-party targets. A warning suppression for
  unavoidable third-party code must be scoped to the third-party sources and
  documented in CMake.

## Testing and Verification

Testing must match the affected behavior and dependency blast radius. A task is
not complete merely because it compiles.

### Standard workspace workflow

Use tests-enabled builds:

~~~bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
colcon test --event-handlers console_cohesion+
colcon test-result --all --verbose
~~~

During development, package-selective commands are acceptable:

~~~bash
colcon build --packages-up-to <package_name> --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select <package_name> --event-handlers console_cohesion+
colcon test-result --all --verbose
~~~

A package-selective build validates dependencies, not downstream consumers.
Changes to public APIs, protocol, exported CMake, or shared runtime behavior
must also build and test affected downstream packages or the full workspace.

Documentation-only changes do not require a C++ rebuild unless they modify
commands, configuration examples, generated inputs, API contracts, or test
instructions that need validation.

### Change-specific verification

- New independently testable behavior requires unit tests.
- A bug fix should include a regression test that fails before the fix whenever
  practical.
- Protocol and codec changes require round-trip, malformed-input, limit,
  allocation, performance, and server/client integration coverage as
  applicable.
- Configuration generator changes require generator regression tests, runtime
  parser tests, and an installed consumer test. Schema changes require valid,
  invalid, unknown-field, duplicate-field, range, and CLI-over-YAML precedence
  coverage as applicable.
- Resource registration changes require valid resources, unknown resources,
  duplicates, command ownership conflicts, replacement, disconnect, reconnect,
  unregister, and expiry coverage as applicable.
- Concurrency changes require normal operation, timeout, delayed or lost input,
  shutdown during active work, callback teardown, and repeated Start/Stop
  coverage.
- Changes to installed APIs or CMake exports require the affected package
  consumer tests.
- Server GUI or Sensor feature-boundary changes require the GUI ON/OFF by Sensor
  ON/OFF build matrix using MUJOCO_SIM_RUN_BUILD_MATRIX_TESTS.
- GUI behavior requires Xvfb-backed tests when no display is available.
  Headless Sensor behavior must also be tested with DISPLAY and WAYLAND_DISPLAY
  unset.
- Changes to Simulation Core or Adapter SDK complexity or public documentation
  require the applicable quality and Doxygen gates described in
  tools/quality/README.md.
- Concurrency-sensitive Server changes require the applicable ThreadSanitizer
  scope. Use the checked-in suppression only for stacks entering the prebuilt,
  uninstrumented eCAL library.
- High-frequency Core or codec changes require the checked-in allocation and
  performance tests. Record benchmark environment and results when updating a
  performance baseline.

Runtime, lifecycle, or concurrency changes must include a bounded run that
exercises initialization, steady operation, and shutdown. Select and report an
explicit duration or tick count appropriate to the risk. Memory-growth claims
must include the measurement method and observed result; do not describe a
short smoke test as a memory-stability test.

Always report the exact build and test commands executed. If an environment
prevents a required check, report the missing dependency or environmental
constraint and the unverified scope. Never report an unexecuted or stale test
result as a current pass.

## Change Scope and Review

- Keep changes focused on the requested task.
- Do not combine an unrelated architectural refactor with a narrow feature or
  bug fix.
- If an unrelated issue blocks the task, make only the minimum supporting
  change and document why it is required.
- Do not rename public APIs, packages, executable names, protocol fields,
  resource names, or configuration keys without an explicit requirement.
- Preserve user changes already present in the worktree.

Before declaring work complete:

1. Re-read the original requirements and current architecture documentation.
2. Review package responsibility, dependency direction, and installed API
   impact.
3. Review ownership, lifetime, synchronization, callback, and shutdown behavior.
4. Check every changed first-party header for complete and consistent English
   and Chinese documentation.
5. Format changed first-party C++ code and inspect newly introduced warnings.
6. Run the required package, integration, consumer, matrix, quality, sanitizer,
   and runtime checks for the affected scope.
7. Inspect all test output and resolve new failures.
8. Review the final diff, including generated-file, third-party, and package
   metadata changes.
9. Confirm that no debugging code, local paths, credentials, generated build
   output, logs, screenshots, core dumps, or temporary configuration files are
   included.
10. Summarize behavioral changes, compatibility impact, tests run, and any
    remaining unverified risk.
