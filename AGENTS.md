# Project Knowledge Base

**Generated:** 2026-05-16
**Commit:** 43929d3
**Branch:** main

## Overview

ROS 2/colcon workspace for latency-management experiments on an Autoware-inspired reference system. Core stack is C++17 ROS 2 packages, custom IDL messages, benchmark/report Python scripts, and a vendored/modified `rclcpp` with PICAS and latency-management executor hooks.

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
| Understand benchmark goals and node graph | `README.md`, `autoware_reference_system/README.md` | Root describes reference-system rules; Autoware README has KPIs and run flow. |
| Change executable variants | `autoware_reference_system/src/ros2/executor/`, `autoware_reference_system/CMakeLists.txt` | Add executables to `TEST_TARGETS` for benchmark tests. |
| Change graph topology | `autoware_reference_system/include/autoware_reference_system/autoware_system_builder.hpp` | Topics are usually named after publishing nodes; PICAS priorities are wired conditionally. |
| Change timing/workload | `autoware_reference_system/include/autoware_reference_system/system/timing/` | Use `number_cruncher_benchmark` to calibrate platform-specific limits. |
| Change reusable node behavior | `reference_system/include/reference_system/nodes/rclcpp/` | Sensor, transform, fusion, cyclic, command, and intersection primitives. |
| Change message payloads | `reference_interfaces/msg/`, `reference_system/include/reference_system/msg_types.hpp` | Current benchmark default is `Message4kb`. |
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
| `number_cruncher` | function | `reference_system/include/reference_system/number_cruncher.hpp` | Synthetic CPU workload for deterministic processing time. |
| `rclcpp::Executor` | class | `ros2-picas/rclcpp/src/rclcpp/executor.cpp` | PICAS callback-priority selection is patched here. |
| `SingleThreadedExecutor::spin_rt` | method | `ros2-picas/rclcpp/src/rclcpp/executors/single_threaded_executor.cpp` | Real-time priority / CPU-affinity spin path. |

## Conventions

- Build as a ROS 2 workspace from the workspace root that contains this repo under `src/`; docs assume `colcon_ws/src/reference-system-latency-management` plus `ros2_tracing` in the same workspace.
- C++ packages default to C++17 and compile with `-Wall -Wextra -Wpedantic`; modified `rclcpp` also enables conversion and virtual-dtor warnings.
- `PICAS`, `LAME`, `RUN_BENCHMARK`, `TEST_PLATFORM`, `SKIP_TRACING`, and `ALL_RMWS` are CMake-time feature switches, not runtime flags.
- Benchmark runs use one fixed message type for all nodes; update both IDL and `msg_types.hpp` when changing payload size.
- New benchmark executables must be added in `autoware_reference_system/CMakeLists.txt`; otherwise they build only if explicitly added and are not benchmarked.
- Apache 2.0 headers are present on project code copied from Apex.AI/ROS 2; preserve license headers in touched files.

## Anti-Patterns

- Do not treat `ros2-picas/rclcpp/` like ordinary app code; it is a forked upstream dependency with broad API surface.
- Do not run benchmark tests without required Python deps, LTTng, and `ros2_tracing` overlay; tests are documented to fail when these are missing.
- Do not compare results across runs with different message types, timing limits, RMW implementations, or platform configuration.
- Do not count fusion-node dropped messages as requirements failures; README states their input-frequency behavior can drop by design.
- Do not assume CI covers Humble exactly: workflow matrix is foxy/galactic/rolling, while root README says the project uses Humble as base.

## Commands

```bash
# from colcon workspace root, after sourcing ROS
rosdep install --from-paths src --ignore-src -y
python3 -m pip install psrecord bokeh networkx numpy pandas

# standard benchmark build
colcon build --cmake-args -DRUN_BENCHMARK=TRUE

# platform-gated benchmark build
colcon build --cmake-args -DRUN_BENCHMARK=TRUE -DTEST_PLATFORM=TRUE

# PICAS executor support
colcon build --cmake-args -DRUN_BENCHMARK=TRUE -DPICAS=TRUE

# latency-management executor support
colcon build --cmake-args -DRUN_BENCHMARK=TRUE -DLAME=TRUE

# after build
source install/local_setup.bash
colcon test

# calibrate synthetic workload
ros2 run autoware_reference_system number_cruncher_benchmark
```

## Notes

- Test output locations: callback traces in `${ROS_HOME:-~/.ros}/tracing`, memory traces in `${ROS_HOME:-~/.ros}/memory`, std/log parsing under `log/latest_test/autoware_reference_system`.
- PICAS examples requiring Linux real-time priorities need `/etc/security/limits.conf` `rtprio 99` entries and a reboot; examples commonly run with `sudo`.
- `content/img/*.svg` and `.pdf` document the graph; regenerate through `autoware_reference_system/test/generate_graph.sh` when topology changes.
