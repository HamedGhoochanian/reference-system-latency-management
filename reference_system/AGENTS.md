# reference_system

## Overview

Header-only ROS 2 package containing reusable node primitives and sample metadata helpers used to assemble benchmark reference systems.

## Structure

```text
reference_system/
|-- include/reference_system/nodes/rclcpp/ # sensor/transform/fusion/cyclic/command/intersection nodes
|-- include/reference_system/system/       # type aliases for system construction
|-- include/reference_system/msg_types.hpp # benchmark message alias
|-- include/reference_system/sample_management.hpp
`-- include/reference_system/number_cruncher.hpp
```

## Where To Look

| Task | Location | Notes |
|------|----------|-------|
| Change node settings API | `include/reference_system/nodes/settings.hpp` | Autoware builder constructs these settings directly. |
| Change node behavior | `include/reference_system/nodes/rclcpp/*.hpp` | Classes derive from `rclcpp::Node` and are header-only. |
| Change sample history / dropped count | `include/reference_system/sample_management.hpp` | Used by all node primitives to propagate benchmark metadata. |
| Change message type | `include/reference_system/msg_types.hpp` | Keep synchronized with `reference_interfaces/msg/*.idl`. |
| Change synthetic workload | `include/reference_system/number_cruncher.hpp` | Used by processing nodes and benchmark calibration. |

## Conventions

- This package installs an `INTERFACE` target only; avoid adding `.cpp` implementation files unless the package model changes.
- Node primitives use QoS depth `1` and loaned messages to model newest-sample behavior.
- `#ifdef PICAS` fields set `callback_priority` on timers/subscriptions; non-PICAS builds must remain valid without those members.
- Output messages should keep sample lineage via `set_sample`, `merge_history_into_sample`, and missed-sample accounting helpers.

## Anti-Patterns

- Do not add Autoware-specific topology or node names here; keep this package reusable for other reference systems.
- Do not change `message_t` without checking every benchmark package and report parser that assumes a single fixed message type.
