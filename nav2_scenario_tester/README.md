# nav2_scenario_tester

Reusable integration-test framework for Nav2.

## Running a test

```bash
launch_test nav2_scenario_tester/scenarios/warehouse_aisles/test.py
```

Each scenario directory contains a `test.py` and a `test_cases.yaml`.

The `test_cases.yaml` defines the map, params, and test cases:

```yaml
map: warehouse_aisles.yaml              # filename in maps/
params: params.yaml                      # filename in params/
behavior_tree: navigate_w_replanning_only_if_goal_is_updated.xml  # optional

test_cases:
  - name: narrowly_passing_next_to_obstacle
    initial_pose: {x: 86.09, y: 33.76, yaw: -1.54}
    goal_pose: {x: 86.35, y: 22.57, yaw: -1.60}
    timeout: 60.0
    obstacles:
      - [[87.33, 28.45], [86.79, 28.48], [87.13, 28.14]]
    limits:
      center_cost: [null, 200]
```



## Launching the test environment

Launch the nav2 stack with loopback simulation without running any test:

```bash
ros2 launch nav2_scenario_tester loopback_test.launch.py \
    params_file:=params.yaml map:=warehouse_aisles.yaml rviz:=true
```

| Argument | Default | Description |
|---|---|---|
| `params_file` | `params.yaml` | Filename looked up in `params/` |
| `rviz` | `false` | Launch RViz |

## Test designer

Launch the RViz test designer to visually create and edit test cases:

```bash
ros2 launch nav2_scenario_tester test_designer.launch.py
```

The designer provides four RViz plugins:

| Plugin | Type | Description |
|---|---|---|
| `NavTestDesignerPanel` | Panel | Table of test cases with save/load YAML |
| `StartPoseTool` | Tool | Click the map to set a test's start pose |
| `GoalPoseTool` | Tool | Click the map to set a test's goal pose |
| `ObstacleTool` | Tool | Click to place obstacle polygon vertices |
