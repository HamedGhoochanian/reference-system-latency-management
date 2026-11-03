# RSLM source guidance

## Overview

ROS 2/colcon workspace for latency-management experiments on an Autoware-inspired reference system. Core stack is C++17 ROS 2 packages, custom IDL messages, benchmark/report Python scripts, and a vendored/modified `rclcpp` with PICAS and latency-management executor hooks.

Use ROS 2 Humble. Read applicable package guidance before changing files. When embedded in the thesis repository, the parent guidance applies. Keep project chronology and temporary locations out of agent instructions. Use the owning study's objective and experiment reports when interpreting benchmark results.

Use read-only Git queries to establish this checkout's root, revision, and local changes. Do not infer its source identity from a branch name, colcon path, or another checkout. Preserve uncommitted work and historical measurement inputs.

## Structure

```text
reference-system-latency-management/
|-- autoware_reference_system/ # concrete Autoware benchmark graph, executables, tests, reports
|-- reference_system/          # reusable header-only reference node primitives
|-- reference_interfaces/      # fixed-size benchmark messages
|-- ros2-picas/                # modified rclcpp plus PICAS examples
|-- scripts/                   # host tuning helpers for CPU frequency / cruncher limit
|-- content/img/               # generated architecture diagrams used by READMEs
`-- .github/workflows/         # colcon CI for autoware_reference_system
```

## Where To Look

| Task | Location | Notes |
|------|----------|-------|
| Understand benchmark goals and node graph | `README.md`, `autoware_reference_system/README.md` | Distinguish inherited benchmark goals from local fork behavior and thesis counting rules. Read builder and node source for actual semantics. |
| Change executable variants | `autoware_reference_system/src/ros2/executor/`, `autoware_reference_system/CMakeLists.txt` | Add executables to `TEST_TARGETS` for benchmark tests. |
| Change graph topology | `autoware_reference_system/include/autoware_reference_system/autoware_system_builder.hpp` | Topics are usually named after publishing nodes; PICAS priorities are wired conditionally. |
| Change timing/workload | `autoware_reference_system/include/autoware_reference_system/system/timing/` | Use `number_cruncher_benchmark` to calibrate platform-specific limits. |
| Change reusable node behavior | `reference_system/include/reference_system/nodes/rclcpp/` | Sensor, transform, fusion, cyclic, command, and intersection primitives. |
| Change message payloads | `reference_interfaces/msg/`, `reference_system/include/reference_system/msg_types.hpp` | Read the alias and IDL together. Preserve provenance and parser compatibility. |
| Change modified ROS client library behavior | `ros2-picas/rclcpp/` | Treat as upstream-derived `rclcpp`; isolate local PICAS/LAME changes. |
| Inspect generated reports/traces | `autoware_reference_system/test/` | Python scripts parse LTTng traces, std logs, memory data, and dropped messages. |

## Code Map

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `create_autoware_nodes` | template function | `autoware_reference_system/include/autoware_reference_system/autoware_system_builder.hpp` | Builds the benchmark node graph and topic wiring. |
| `TimingConfig::...` | config constants | `autoware_reference_system/include/autoware_reference_system/system/timing/*.hpp` | Controls publish periods and number-cruncher work. |
| `callback::priority::Default` | config constants | `autoware_reference_system/include/autoware_reference_system/system/priority/default.hpp` | PICAS callback priority map. |
| `nodes::rclcpp_system::*` | node classes | `reference_system/include/reference_system/nodes/rclcpp/` | Header-only ROS 2 node primitives used by benchmark systems. |
| `message_t` | alias | `reference_system/include/reference_system/msg_types.hpp` | Single message type used during a benchmark run. |
| `number_cruncher` | function | `reference_system/include/reference_system/number_cruncher.hpp` | Synthetic CPU work. Elapsed time depends on the host and contention. |
| `rclcpp::Executor` | class | `ros2-picas/rclcpp/src/rclcpp/executor.cpp` | PICAS callback-priority selection is patched here. |
| `SingleThreadedExecutor::spin_rt` | method | `ros2-picas/rclcpp/src/rclcpp/executors/single_threaded_executor.cpp` | Real-time priority / CPU-affinity spin path. |

## Conventions

- Select the source and colcon workspace explicitly. Container builds use the owning repository's build route. Do not assume a fixed `~/colcon_ws` path.
- C++ packages default to C++17 and compile with `-Wall -Wextra -Wpedantic`; modified `rclcpp` also enables conversion and virtual-dtor warnings.
- `PICAS`, `LAME`, `RUN_BENCHMARK`, `TEST_PLATFORM`, `SKIP_TRACING`, and `ALL_RMWS` are CMake-time feature switches, not runtime flags.
- Benchmark runs use one fixed message type for all nodes; update both IDL and `msg_types.hpp` when changing payload size.
- New benchmark executables must be added in `autoware_reference_system/CMakeLists.txt`; otherwise they build only if explicitly added and are not benchmarked.
- Apache 2.0 headers are present on project code copied from Apex.AI/ROS 2; preserve license headers in touched files.

## Anti-Patterns

- Do not treat `ros2-picas/rclcpp/` like ordinary app code; it is a forked upstream dependency with broad API surface.
- Do not run benchmark tests without required Python deps, LTTng, and `ros2_tracing` overlay; tests are documented to fail when these are missing.
- Do not compare results across runs with different message types, timing limits, RMW implementations, or platform configuration.
- Distinguish cached-input reuse, dropped workload messages, missing completions, and telemetry loss. Apply the owning study's counting rules, not inherited KPI language.
- Read the actual CI matrix before relying on distribution coverage. Offline checks do not establish Humble runtime qualification or research benefit.

## Setup and execution boundaries

Read the package README and CMake inputs for build switches and test generation. These are interfaces, not permission to build or start benchmark workloads.
Use `uv` for Python environments and scripts, and `uvx` for Python tools. Do not globally install unpinned Python packages or shadow ROS system dependencies.
For thesis-host work, follow the [host dependency constraints](../../notes/uni_laptop_setup.md). If this is a standalone checkout, obtain the target host's dependency specification before installation.
Builds, calibration, benchmark tests, Docker actions, and host tuning need the scope allowed by the active assignment. Preserve failed runs and original evidence. Do not change image identities or source pins to make an old campaign accept new code.

## Notes

- Test output locations: callback traces in `${ROS_HOME:-~/.ros}/tracing`, memory traces in `${ROS_HOME:-~/.ros}/memory`, std/log parsing under `log/latest_test/autoware_reference_system`.
- Real-time priority and affinity operations need explicit host configuration and permission. Example use of sudo is not authorization to modify the host or reboot it.
- `content/img/*.svg` and `.pdf` document the graph; regenerate through `autoware_reference_system/test/generate_graph.sh` when topology changes.
