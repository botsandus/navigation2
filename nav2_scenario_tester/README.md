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

### Launching a test

Tests use the `launch_test` runner from `launch_testing`. Run a scenario
directly from the command line:

```bash
launch_test path/to/test.py
```

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
| `OdometryMetrics` | `/odom` topic | `distance_travelled`, `elapsed_time`, `max_speed` |
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



## RViz test designer


### `test_designer.launch.py`

Launches map\_server + lifecycle\_manager + RViz2 with the test-designer panel
and tools for visually creating and editing test cases.

```bash
ros2 launch nav2_scenario_tester test_designer.launch.py
```

### Using the panel

1. **Add / remove test cases** — Click **Add Test Case** to create a new row.
   Select a row and click **Remove Selected** to delete it.
   Edit the name and pose values directly in the table cells.

2. **Set poses from the map** — Select a row, then click **Set Start** or
   **Set Goal**. Click and drag on the map to place the pose (drag direction
   sets the yaw). The table updates automatically.

3. **Draw obstacles** — Select a row and click **Draw Obstacle**. Left-click
   on the map to place vertices. Click **Finish** to close the polygon
   (minimum 3 vertices) or **Cancel** to discard it. Use **Remove Last
   Obstacle** to delete the most recent polygon from the selected test case.

4. **Save / Load** — Click **Save YAML** or **Load YAML** to export or import
   test cases via a file dialog Each test case is visualised with coloured arrows and obstacle
   outlines on the `/nav_test_designer/markers` topic.
