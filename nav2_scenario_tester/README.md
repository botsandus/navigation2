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

## Running tests against an already-running environment

If you have already launched the environment (e.g. in a separate terminal), you
can run only the test assertions without relaunching the nav2 stack:

```bash
# Terminal 1 — start the environment
ros2 launch nav2_scenario_tester loopback_test.launch.py \
    params_file:=params.yaml map:=warehouse_aisles.yaml rviz:=true

# Terminal 2 — run tests only
NAV2_SCENARIO_SKIP_LAUNCH=1 launch_test nav2_scenario_tester/scenarios/warehouse_aisles/test.py
```

Setting `NAV2_SCENARIO_SKIP_LAUNCH=1` skips launching the nav2 stack and
connects directly to the running environment. The runner still waits for
`bt_navigator` to become active before executing the first test case. In this
mode, test teardown does not shut down lifecycle managers, so the externally
launched environment stays up after tests complete.

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
