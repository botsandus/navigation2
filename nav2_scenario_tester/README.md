# nav2_scenario_tester

Reusable integration-test framework for Nav2.
Provides a persistent-simulation test runner, YAML-driven parameterised test cases,
live metrics collection, and an RViz visual test designer.

## Quick start

### YAML-driven launch test

Define test cases in YAML:

```yaml
test_cases:
  - name: short_forward
    initial_pose: {x: -2.0, y: -0.5, yaw: 0.0}
    goal_pose: {x: 0.0, y: -0.5, yaw: 0.0}
    timeout: 60.0
    limits:
      distance_travelled: [null, 4.0]
```

Write a `launch_testing` test in Python:

```python
from nav2_scenario_tester import (
    CostmapMetrics, load_test_suite, NavTestRunner, OdometryMetrics, PlanMetrics,
)

TEST_SUITE = load_test_suite('path/to/test_cases.yaml')

# In generate_test_description(), include loopback_test.launch.py
# (or gazebo_test.launch.py) with your map and params_file.

# In the test class:
runner = NavTestRunner(
    metrics_collectors=[OdometryMetrics(), PlanMetrics(), CostmapMetrics()],
)
for tc in TEST_SUITE.cases:
    result = runner.run(
        initial_pose=tc.initial_pose,
        goal_pose=tc.goal_pose,
        timeout=tc.timeout,
        limits=tc.limits,
    )
    assert result.success
```

See [test/example_parameterized_test.py](test/example_parameterized_test.py)
for a complete working example.

## YAML test case format

```yaml
test_cases:
  - name: cross_warehouse
    initial_pose: {x: 9.0, y: 10.5, yaw: 0.0}
    goal_pose: {x: 10.0, y: 10.5, yaw: 0.0}
    timeout: 90.0
    obstacles:                              # optional
      - [[9.5, 10.3], [9.5, 10.7], [9.7, 10.7], [9.7, 10.3]]
    limits:                                 # optional, [min, max] per metric
      distance_travelled: [null, 4.0]       # null = no bound
      plan_length: [2.0, 6.0]
      max_footprint_cost: [null, 252]
```

## Metrics collectors

| Collector | Source | Metrics |
|---|---|---|
| `OdometryMetrics` | `/odom` topic | `distance_travelled`, `elapsed_time`, `max_linear_speed`, `max_angular_speed` |
| `PlanMetrics` | `/plan` topic | `plan_length`, `plan_waypoints`, `plan_count` |
| `CostmapMetrics` | `/local_costmap/get_costs` service | `max_footprint_cost`, `avg_footprint_cost`, `samples` |

All collectors support live limit checking: if a `[min, max]` bound from the
YAML is breached mid-navigation, the goal is cancelled early.

### Custom collectors

Subclass `MetricsCollector` and implement `topics()`, `on_message()`, and
`report()`. Optionally override `check()` for live limit enforcement and
`setup(node)` if you need service clients or timers.

```python
from nav2_scenario_tester import MetricsCollector

class MyMetrics(MetricsCollector):
    def topics(self):
        return ['/my_topic']

    def on_message(self, topic, msg):
        ...

    def report(self):
        return {'my_metric': self._value}
```

## Launch files

### `loopback_test.launch.py`

Spins up a loopback simulator, map_server, and Nav2 stack for testing.
Intended for use via `IncludeLaunchDescription` in user test files.

| Argument | Default | Description |
|---|---|---|
| `params_file` | built-in | Nav2 parameters YAML |
| `map` | built-in empty map | Map YAML |
| `use_sim_time` | `true` | Use simulation clock |
| `namespace` | `''` | Top-level namespace |
| `log_level` | `info` | ROS log level |

### `gazebo_test.launch.py`

Spins up Gazebo, spawns a robot, starts map_server, and Nav2 stack for testing.

| Argument | Default | Description |
|---|---|---|
| `params_file` | built-in | Nav2 parameters YAML |
| `map` | built-in empty map | Map YAML |
| `use_sim_time` | `true` | Use simulation clock |
| `namespace` | `''` | Top-level namespace |
| `log_level` | `info` | ROS log level |
| `world` | built-in | Gazebo world SDF/xacro |
| `robot_sdf` | built-in | Robot SDF for Gazebo spawn |
| `x_pose`, `y_pose`, `z_pose`, `yaw` | `-2.0`, `-0.5`, `0.01`, `0.0` | Robot spawn pose |
| `headless` | `true` | Run Gazebo in server-only mode |

### `test_designer.launch.py`

Launches map\_server + lifecycle\_manager + RViz2 with the test-designer panel
and tools for visually creating and editing test cases.

```bash
ros2 launch nav2_scenario_tester test_designer.launch.py
```

## RViz test designer

The package provides four RViz plugins:

| Plugin | Type | Description |
|---|---|---|
| `NavTestDesignerPanel` | Panel | Table of test cases with save/load YAML |
| `StartPoseTool` | Tool | Click the map to set a test's start pose |
| `GoalPoseTool` | Tool | Click the map to set a test's goal pose |
| `ObstacleTool` | Tool | Click to place obstacle polygon vertices |

Loading a YAML with a `map:` field will hot-swap the map via the
`/map_server/load_map` service.
