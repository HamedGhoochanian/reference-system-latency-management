# ros2-picas/rclcpp

## Overview

Forked ROS 2 `rclcpp` package with local executor scheduling changes for PICAS / latency-management experiments.

## Structure

```text
ros2-picas/rclcpp/
|-- include/rclcpp/          # public API headers; PICAS adds callback-priority fields
|-- src/rclcpp/              # implementation, executor scheduling, node internals
|-- src/rclcpp/executors/    # single/multi/static executor implementations
|-- test/                    # upstream-style rclcpp tests and benchmarks
|-- resource/*.em            # templates expanded into generated headers at build time
`-- doc/, QUALITY_DECLARATION.md
```

## Where To Look

| Task | Location | Notes |
|------|----------|-------|
| Change callback-priority selection | `src/rclcpp/executor.cpp` | PICAS logic chooses highest-priority executable when enabled. |
| Change single-threaded RT behavior | `src/rclcpp/executors/single_threaded_executor.cpp`, `include/rclcpp/executors/single_threaded_executor.hpp` | Includes `spin_rt`, scheduler attributes, and CPU affinity. |
| Change multi-threaded RT behavior | `src/rclcpp/executors/multi_threaded_executor.cpp`, `include/rclcpp/executors/multi_threaded_executor.hpp` | Uses `cpus` and `rt_attr` for worker threads. |
| Change callback priority storage | `include/rclcpp/timer.hpp`, `subscription.hpp`, `client.hpp`, `service.hpp`, `waitable.hpp` | Fields are guarded by `#ifdef PICAS`. |
| Change generated logging/interface headers | `resource/*.em`, `CMakeLists.txt` | Build expands templates into generated headers. |

## Conventions

- Keep local scheduling changes behind `#ifdef PICAS` or the relevant feature define unless intentionally changing default `rclcpp` behavior.
- Build with `--allow-overriding rclcpp` when this package overlays a ROS distribution.
- Preserve upstream API compatibility and tests unless the benchmark explicitly requires a divergence.
- `PICAS_DEBUG` is toggled in `include/rclcpp/cb_sched.hpp`; leave disabled for normal benchmark runs.

## Anti-Patterns

- Do not make broad formatting-only changes in this subtree; diffs should isolate behavioral changes from upstream `rclcpp`.
- Do not assume real-time scheduler calls work without OS permissions and CPU availability; PICAS examples require `rtprio` setup.
- Do not edit generated build outputs for `logging.hpp` or interface traits; change templates or source headers instead.
