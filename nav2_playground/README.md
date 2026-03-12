# nav2_playground

A lightweight navigation playground for testing controllers, planners, and
behaviors using a loopback simulator — no physics engine or real robot required.

## What It Does

- Launches the Nav2 navigation stack (controller, planner, BT navigator,
  behavior server, smoother) with a **loopback simulator** that converts
  `cmd_vel` → odometry/TF/fake laser scans
- Loads a map via `map_server` with static-layer-only costmaps
- Provides a test runner for automated navigation scenario testing
- Pose setting via RViz "2D Pose Estimate" or any tool that publishes to
  `/initialpose`
- Goal setting via RViz "2D Nav Goal" or any tool that publishes to
  `/goal_pose`

## Quick Start

```bash
# Build
colcon build --packages-select nav2_playground

# Source
source install/setup.bash

# Launch with the default tb3_sandbox map
ros2 launch nav2_playground playground.launch.py

# Launch with a custom map
ros2 launch nav2_playground playground.launch.py map:=/path/to/your/map.yaml
```

Once launched:
1. In RViz, click **"2D Pose Estimate"** to place the robot
2. Click **"2D Nav Goal"** to send a navigation goal
3. Watch the robot navigate via the loopback simulator

## Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `map` | `nav2_bringup/maps/tb3_sandbox.yaml` | Path to map YAML file |
| `params_file` | `nav2_playground/config/playground_params.yaml` | Path to nav2 params file |
| `use_rviz` | `True` | Whether to launch RViz |
| `rviz_config_file` | `nav2_bringup/rviz/nav2_default_view.rviz` | RViz config file |
| `autostart` | `true` | Auto-activate lifecycle nodes |
| `use_sim_time` | `true` | Use simulated clock from loopback sim |
| `log_level` | `info` | Log level for navigation nodes |
| `namespace` | `''` | Top-level namespace |

## Customizing Parameters

Copy and modify the default params file:

```bash
cp $(ros2 pkg prefix nav2_playground)/share/nav2_playground/config/playground_params.yaml my_params.yaml
# Edit my_params.yaml — change controller, planner, costmap config, etc.
ros2 launch nav2_playground playground.launch.py params_file:=$(pwd)/my_params.yaml
```

The params file configures:
- **controller_server** — DWB local planner (swap for MPPI, RPP, etc.)
- **planner_server** — NavFn global planner (swap for SmacPlanner, etc.)
- **bt_navigator** — Behavior tree navigator
- **behavior_server** — Recovery behaviors (spin, backup, wait, drive_on_heading)
- **smoother_server** — Path smoother
- **local_costmap / global_costmap** — Static layer only (add inflation, obstacle layers as needed)
- **loopback_simulator** — Update rate, frame IDs, scan parameters

## Automated Testing

### Run the built-in test scenarios

```bash
# With the playground already running:
ros2 run nav2_playground playground_test_runner
```

### Write your own test cases

```python
from nav2_playground.test_runner import NavTestCase, run_test, run_all_tests

my_tests = [
    NavTestCase(
        name='my_scenario',
        start_x=0.0, start_y=0.0, start_yaw=0.0,
        goal_x=2.0, goal_y=1.0, goal_yaw=1.57,
        timeout_sec=30.0,
    ),
]

# In a ROS2 context with BasicNavigator:
results = run_all_tests(navigator, my_tests)
```

### Run with launch_testing (full integration)

```bash
launch_test $(ros2 pkg prefix nav2_playground)/share/nav2_playground/tests/test_playground_scenarios.py
```

## Architecture

```
                    /initialpose (RViz / Foxglove)
                          │
                          ▼
┌──────────────────────────────────────────────────┐
│              loopback_simulator                   │
│  cmd_vel → integrate → publish odom + TF + scan  │
│  TF: map → odom → base_footprint                 │
│  Also publishes /clock                            │
└──────────┬───────────────────────────────────────┘
           │ /cmd_vel                    ▲ /odom, /scan, /tf
           │                             │
┌──────────┴─────────────────────────────┴─────────┐
│              Navigation Stack                     │
│  controller_server ──► /cmd_vel                   │
│  planner_server                                   │
│  bt_navigator                                     │
│  behavior_server                                  │
│  smoother_server                                  │
│  lifecycle_manager_navigation                     │
└──────────────────────────────────────────────────┘
           ▲ /map
           │
┌──────────┴───────────────────────────────────────┐
│  map_server + lifecycle_manager_map_server        │
└──────────────────────────────────────────────────┘
```

## What's NOT Included (Phase 1)

- No velocity smoother or collision monitor (controller output goes directly to loopback sim)
- No AMCL or localization (loopback sim publishes `map→odom` directly)
- No composition (all standalone nodes for easy debugging)
- No obstacle/voxel costmap layers (static layer only)
- No robot_state_publisher / URDF
