# Maps

By default, `navigation_test.launch.py` uses the `tb3_sandbox` map from `nav2_bringup`.

To use a custom map, pass the `map` launch argument:

```bash
ros2 launch nav2_navigation_test navigation_test.launch.py map:=/path/to/your/map.yaml
```

Place custom maps in this directory if bundling them with the package.
