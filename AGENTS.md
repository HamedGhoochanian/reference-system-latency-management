# Agent Instructions

## Build Commands

```bash
# Standard build with benchmark tests
colcon build --cmake-args -DRUN_BENCHMARK=TRUE

# Build with PICAS executor
colcon build --cmake-args -DRUN_BENCHMARK=TRUE -DPICAS=TRUE

# Build with Latency Management executor (LAME)
colcon build --cmake-args -DRUN_BENCHMARK=TRUE -DLAME=TRUE
```

## Running Tests

```bash
source install/local_setup.bash
colcon test
```

## Required Dependencies

- Python: `psrecord bokeh networkx numpy pandas`
- ROS: `rosdep install --from-paths src --ignore-src -y`
- Tracing: LTTng and ros2_tracing (must be in same workspace as this repo)

## PICAS-Specific Setup

Edit `/etc/security/limits.conf` then reboot:
```
<userid>    hard    rtprio  99
<userid>    soft    rtprio  99
```

## Package Structure

- `autoware_reference_system/` - Main reference system with nodes, executors, tests
- `reference_system/` - Core node definitions
- `reference_interfaces/` - Message definitions (Message4kB)
- `ros2-picas/` - Modified rclcpp with latency management

## Key Executables

- `number_cruncher_benchmark` - Run to calibrate processing time for your platform

## Test Output Locations

- CPU/Memory: `${ROS_HOME}/memory` (default: `~/.ros/memory`)
- Latency/Jitter: `log/latest_test/autoware_reference_system`
- Tracing: `${ROS_HOME}/tracing` (default: `~/.ros/tracing`)