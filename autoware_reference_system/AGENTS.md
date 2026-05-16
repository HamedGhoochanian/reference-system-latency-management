# autoware_reference_system

## Overview

Concrete Autoware-style benchmark package: builds executor variants, defines the fixed node graph, and owns launch tests plus trace/report generation.

## Structure

```text
autoware_reference_system/
|-- include/autoware_reference_system/ # graph builder plus timing/priority configs
|-- src/ros2/executor/                 # runnable executor variants under test
|-- cmake/                             # macros that generate benchmark tests/reports
|-- test/                              # launch tests and Python trace/report parsers
|-- scripts/                           # benchmark orchestration shell helpers
`-- autoware_reference_system.dot      # graphviz source for architecture diagram
```

## Where To Look

| Task | Location | Notes |
|------|----------|-------|
| Add or change benchmark executable | `src/ros2/executor/`, `CMakeLists.txt` | Add executable with `ament_auto_add_executable`; add to `TEST_TARGETS` for benchmark sweeps. |
| Change graph nodes/topics | `include/autoware_reference_system/autoware_system_builder.hpp` | Topics mostly equal publisher node names; command nodes terminate output chains. |
| Change synthetic processing times | `include/autoware_reference_system/system/timing/default.hpp` | Values are number-cruncher limits, not wall-time durations. |
| Change benchmark timing assumptions | `include/autoware_reference_system/system/timing/benchmark.hpp` | Includes max input timing used by fusion behavior. |
| Change PICAS priorities | `include/autoware_reference_system/system/priority/default.hpp` | Only compiled when `-DPICAS=TRUE` sets the `PICAS` define. |
| Change report generation | `test/generate_*`, `test/*_duration.py`, `test/dropped_messages.py` | CMake macros template launch tests and call these scripts. |

## Conventions

- `RUN_BENCHMARK` controls the full benchmark matrix; without it only lint/default tests are built.
- `SKIP_TRACING=TRUE` removes callback trace tests but leaves memory/std trace paths active.
- `ALL_RMWS=ON` tests all discovered RMW implementations; default CMake logic narrows to one implementation.
- `RUN_TIMES`, `TRACE_TYPES`, and `TEST_TARGETS` live in `CMakeLists.txt`; update all three deliberately when changing benchmark coverage.
- `ROS_HOME` controls LTTng and memory trace roots; `ROS_LOG_DIR` controls std-log parsing, defaulting into colcon `log/latest_test`.
- Platform-specific calibration is expected: run `number_cruncher_benchmark`, then update timing config limits for the target hardware.

## Anti-Patterns

- Do not add an executor source without wiring it into benchmark tests when it should appear in reports.
- Do not interpret fusion-node message drops the same as transform-node drops; the README calls out different expected behavior.
- Do not move `ros2_tracing` out of the workspace used for this package; tests depend on the overlay being sourced from the same workspace.
