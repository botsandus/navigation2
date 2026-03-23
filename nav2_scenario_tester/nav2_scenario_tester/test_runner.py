r"""
NavTestRunner -- reusable test runner for Nav2 BT + planner + controller integration tests.

Designed for persistent simulation: create one runner in setUpClass, call run()
with different start/goal poses per test method, and shut down in tearDownClass.

Usage from a launch_testing test file::

    from nav2_scenario_tester import NavTestRunner
    from nav2_scenario_tester.test_runner import make_pose

    class TestNavigation(unittest.TestCase):
        @classmethod
        def setUpClass(cls):
            rclpy.init()
            cls.runner = NavTestRunner()

        @classmethod
        def tearDownClass(cls):
            cls.runner.shutdown()
            cls.runner.destroy_node()
            rclpy.shutdown()

        def test_short_path(self):
            result = self.runner.run(
                initial_pose=make_pose(0.0, 0.0),
                goal_pose=make_pose(1.0, 0.0),
            )
            self.assertTrue(result.success)

        def test_longer_path(self):
            result = self.runner.run(
                initial_pose=make_pose(0.0, 0.0),
                goal_pose=make_pose(5.0, 3.0),
                timeout=120.0,
            )
            self.assertTrue(result.success)

"""

from dataclasses import dataclass
import math
import os
import time
from typing import List
import uuid as uuid_module

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Pose, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import PolygonObject
from nav2_msgs.srv import AddShapes, ManageLifecycleNodes, RemoveShapes
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID as UUIDMsg
import yaml


@dataclass
class NavTestResult:
    """Result of a navigation test run."""

    success: bool
    elapsed_time: float
    error_code: int = 0
    error_msg: str = ''
    metrics: dict = None


class NavTestRunner(Node):
    """
    Reusable navigation test runner.

    Designed for persistent simulation: instantiate once, then call run()
    repeatedly with different start/goal poses. The runner waits for the
    nav2 stack to become ready on the first run and reuses it thereafter.
    """

    def __init__(
        self,
        namespace: str = '',
        node_name: str = 'nav_test_runner',
        metrics_collectors: list = None,
    ):
        super().__init__(node_name=node_name, namespace=namespace)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10,
        )

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._add_shapes_client = self.create_client(
            AddShapes, '/vector_object_server/add_shapes',
        )
        self._remove_shapes_client = self.create_client(
            RemoveShapes, '/vector_object_server/remove_shapes',
        )

        self._stack_ready = False
        self._collectors = metrics_collectors or []

        # Set up live subscriptions for all collector topics
        self._topic_collectors = {}
        for collector in self._collectors:
            for topic in collector.topics():
                if topic not in self._topic_collectors:
                    self._topic_collectors[topic] = []
                self._topic_collectors[topic].append(collector)

    # ── Public API ─────────────────────────────────────────────────────

    def run(
        self,
        initial_pose: Pose,
        goal_pose: Pose,
        timeout: float = 30.0,
        settle_time: float = 2.0,
        limits: dict = None,
        fail_fast: bool = False,
        behavior_tree: str = '',
        obstacles: list = None,
    ) -> NavTestResult:
        """
        Execute a navigation test: teleport to initial pose, navigate to goal.

        Can be called multiple times with different poses on the same runner.

        Args:
            initial_pose: Where to place the robot before navigating.
            goal_pose: Navigation goal.
            timeout: Maximum seconds to wait for navigation to complete.
            settle_time: Seconds to wait after setting initial pose for the
                         sim to process the teleport.
            limits: Optional dict of ``{metric: [min, max]}`` for live
                    checks during navigation.
            fail_fast: If True, cancel navigation on the first
                       limit breach. If False (default), log violations
                       but let navigation complete.

        Returns
        -------
        NavTestResult
            With success status and elapsed time.

        """
        # On first run, wait for the nav2 stack to be ready
        if not self._stack_ready:
            self._setup_collector_subscriptions()
            self.wait_for_node_active('bt_navigator')
            self._stack_ready = True

        # Reset collectors for this run
        for c in self._collectors:
            c.reset()

        # Inject obstacles via VectorObjectServer
        if obstacles:
            self._add_obstacles(obstacles)

        # Teleport: publish initial pose and let the sim settle
        self.set_initial_pose(initial_pose)
        time.sleep(settle_time)

        start_time = time.time()

        # Send navigation goal via action
        nav_success, error_code, error_msg = self._navigate_to_pose(
            goal_pose, timeout, limits or {}, fail_fast, behavior_tree,
        )

        elapsed = time.time() - start_time

        # Collect final metrics from live data
        metrics = {}
        for c in self._collectors:
            metrics.update(c.report())

        # Remove obstacles after navigation
        if obstacles:
            self._remove_obstacles()

        return NavTestResult(
            success=nav_success,
            elapsed_time=elapsed,
            error_code=error_code,
            error_msg=error_msg,
            metrics=metrics,
        )

    def set_initial_pose(self, pose: Pose) -> None:
        """Publish a pose to /initialpose (teleports the robot in loopback sim)."""
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f'Publishing initial pose: '
            f'({pose.position.x:.2f}, {pose.position.y:.2f})'
        )
        self.initial_pose_pub.publish(msg)

    def wait_for_node_active(self, node_name: str, timeout: float = 60.0) -> bool:
        """Wait for a lifecycle node to become active."""
        self.get_logger().info(f'Waiting for {node_name} to become active')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)

        if not state_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'{node_service} service not available')
            return False

        req = GetState.Request()
        start = time.time()
        while (time.time() - start) < timeout:
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.get_logger().info(f'{node_name} state: {state}')
                if state == 'active':
                    return True
            time.sleep(2)

        self.get_logger().error(f'{node_name} did not become active within {timeout}s')
        return False

    def shutdown(self) -> None:
        """Shut down the navigation lifecycle managers."""
        self.action_client.destroy()

        for manager_name in [
            'lifecycle_manager_navigation',
            'lifecycle_manager_localization',
        ]:
            service_name = f'{manager_name}/manage_nodes'
            mgr_client = self.create_client(ManageLifecycleNodes, service_name)
            if not mgr_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning(f'{service_name} not available, skipping')
                continue

            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.SHUTDOWN
            future = mgr_client.call_async(req)
            try:
                rclpy.spin_until_future_complete(self, future)
                self.get_logger().info(f'Shut down {manager_name}')
            except Exception as e:
                self.get_logger().error(f'Failed to shut down {manager_name}: {e}')

    def _setup_collector_subscriptions(self) -> None:
        """Create ROS subscriptions for all collector topics."""
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message

        # Let collectors set up service clients, timers, etc.
        for collector in self._collectors:
            collector.setup(self)

        # Discover message types by introspecting the ROS graph
        topic_types = dict(self.get_topic_names_and_types())

        for topic in self._topic_collectors:
            type_names = topic_types.get(topic)
            if not type_names:
                self.get_logger().warning(
                    f'Topic {topic} not found, skipping collector subscription'
                )
                continue
            msg_type = get_message(type_names[0])
            collectors = self._topic_collectors[topic]

            def make_cb(t, cols):
                def cb(msg):
                    for c in cols:
                        c.on_message(t, msg)
                return cb

            self.create_subscription(
                msg_type, topic, make_cb(topic, collectors),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f'Subscribed to {topic} for metrics')

    def _add_obstacles(self, obstacles: list) -> None:
        """Add polygon obstacles to the VectorObjectServer."""
        if not self._add_shapes_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('add_shapes service not available')
            return

        req = AddShapes.Request()
        for polygon_pts in obstacles:
            poly = PolygonObject()
            poly.uuid = UUIDMsg(uuid=list(uuid_module.uuid4().bytes))
            poly.closed = True
            poly.value = 100  # lethal cost
            poly.points = [
                Point32(x=float(pt[0]), y=float(pt[1]), z=0.0)
                for pt in polygon_pts
            ]
            req.polygons.append(poly)

        future = self._add_shapes_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f'Added {len(obstacles)} obstacle(s)')
        else:
            self.get_logger().error('Failed to add obstacles')

    def _remove_obstacles(self) -> None:
        """Remove all obstacles from the VectorObjectServer."""
        if not self._remove_shapes_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('remove_shapes service not available')
            return

        req = RemoveShapes.Request()
        req.all_objects = True
        future = self._remove_shapes_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Removed all obstacles')
        else:
            self.get_logger().error('Failed to remove obstacles')

    # ── Private helpers ────────────────────────────────────────────────

    def _navigate_to_pose(
        self, goal_pose: Pose, timeout: float, limits: dict,
        fail_fast: bool = True, behavior_tree: str = '',
    ) -> tuple[bool, int, str]:
        """Send NavigateToPose action and wait for result with live checks."""
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        if not self.action_client.wait_for_server(timeout_sec=timeout):
            return False, -1, 'NavigateToPose action server not available'

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose = goal_pose
        goal_msg.behavior_tree = behavior_tree

        self.get_logger().info(
            f'Sending goal: ({goal_pose.position.x:.2f}, {goal_pose.position.y:.2f})'
        )
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            return False, -1, 'Goal rejected'

        self.get_logger().info('Goal accepted, waiting for result')
        get_result_future = goal_handle.get_result_async()

        # Spin loop: process callbacks (collectors) and check limits
        violations = {}
        start = time.time()
        while not get_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - start > timeout:
                self.get_logger().error('Navigation timed out')
                goal_handle.cancel_goal_async()
                return False, -1, 'Timeout'

            # Check live metrics against limits
            if limits:
                for c in self._collectors:
                    violation = c.check(limits)
                    if violation:
                        if fail_fast:
                            self.get_logger().error(
                                f'Limit breached: {violation} — cancelling goal'
                            )
                            goal_handle.cancel_goal_async()
                            return False, -2, f'Limit breached: {violation}'
                        metric_name = violation.split('=')[0]
                        if metric_name not in violations:
                            self.get_logger().error(
                                f'Limit breached: {violation}'
                            )
                            violations[metric_name] = violation

        status = get_result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            result = get_result_future.result().result
            return False, result.error_code, result.error_msg

        if violations:
            return False, -2, '; '.join(violations.values())

        self.get_logger().info('Goal succeeded')
        return True, 0, ''


# ── Helpers ─────────────────────────────────────────────────────────────


def make_pose(x: float, y: float, z: float = 0.01, yaw: float = 0.0) -> Pose:
    """Create a Pose message from x, y, z, yaw."""
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    # Convert yaw to quaternion (rotation around Z axis)
    pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.orientation.w = math.cos(float(yaw) / 2.0)
    return pose


# ── Test case data ─────────────────────────────────────────────────────


@dataclass
class TestCase:
    """A single navigation test case loaded from YAML."""

    name: str
    initial_pose: Pose
    goal_pose: Pose
    timeout: float = 60.0
    obstacles: list = None
    limits: dict = None


@dataclass
class TestSuite:
    """A collection of test cases with shared configuration."""

    cases: List[TestCase]
    map_yaml: str = None
    params_file: str = None
    bt_xml: str = None


def _resolve_path(raw: str, yaml_dir: str, pkg_fallback_dir: str) -> str:
    """Resolve a relative path against *yaml_dir* first, then *pkg_fallback_dir*."""
    if not raw:
        return None
    if os.path.isabs(raw):
        return raw
    candidate = os.path.join(yaml_dir, raw)
    if os.path.isfile(candidate):
        return candidate
    candidate = os.path.join(pkg_fallback_dir, raw)
    if os.path.isfile(candidate):
        return candidate
    return raw


def _parse_cases(data: dict) -> List[TestCase]:
    """Parse test_cases list from YAML data."""
    cases = []
    for tc in data.get('test_cases', []):
        ip = tc['initial_pose']
        gp = tc['goal_pose']
        cases.append(TestCase(
            name=tc['name'],
            initial_pose=make_pose(ip['x'], ip['y'], yaw=ip.get('yaw', 0.0)),
            goal_pose=make_pose(gp['x'], gp['y'], yaw=gp.get('yaw', 0.0)),
            timeout=tc.get('timeout', 60.0),
            obstacles=tc.get('obstacles'),
            limits=tc.get('limits'),
        ))
    return cases


def load_test_suite(yaml_path: str) -> TestSuite:
    """
    Load a test suite (map + test cases) from a YAML file.

    If the ``map`` value is a relative path, it is resolved relative to the
    directory containing the YAML file first, then relative to the package's
    installed ``maps/`` directory.

    Expected format::

        map: warehouse.yaml     # relative to YAML dir or package maps/
        map: /absolute/path.yaml  # absolute path used as-is
        params: mppi_tuned.yaml  # relative to YAML dir or package params/
        behavior_tree: navigate_w_replanning_only_if_goal_is_updated.xml  # optional

        test_cases:
          - name: short_forward
            initial_pose: {x: 9.0, y: 10.5, yaw: 0.0}
            goal_pose: {x: 10.0, y: 10.5, yaw: 0.0}
            timeout: 90.0

    Returns
    -------
    TestSuite
        With ``map``, ``params_file``, ``bt_xml`` (str or None) and
        ``cases`` (list of TestCase).

    """
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    from ament_index_python.packages import get_package_share_directory
    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    pkg_share = get_package_share_directory('nav2_scenario_tester')

    map_path = _resolve_path(
        data.get('map'), yaml_dir, os.path.join(pkg_share, 'maps'),
    )
    params_file = _resolve_path(
        data.get('params'), yaml_dir, os.path.join(pkg_share, 'params'),
    )

    bt_xml = data.get('behavior_tree')
    if bt_xml and not os.path.isabs(bt_xml):
        candidate = os.path.join(yaml_dir, bt_xml)
        if os.path.isfile(candidate):
            bt_xml = candidate
        else:
            bt_pkg = get_package_share_directory('nav2_bt_navigator')
            candidate = os.path.join(bt_pkg, 'behavior_trees', bt_xml)
            if os.path.isfile(candidate):
                bt_xml = candidate

    return TestSuite(
        map_yaml=map_path,
        params_file=params_file,
        bt_xml=bt_xml,
        cases=_parse_cases(data),
    )


def load_test_cases(yaml_path: str) -> List[TestCase]:
    """
    Load test cases from a YAML file.

    Expected format::

        test_cases:
          - name: short_forward
            initial_pose: {x: 9.0, y: 10.5, yaw: 0.0}
            goal_pose: {x: 10.0, y: 10.5, yaw: 0.0}
            timeout: 90.0
            obstacles:  # optional
              - [[9.5, 10.3], [9.5, 10.7], [9.7, 10.7], [9.7, 10.3]]
            limits:  # optional, [min, max] per metric (null = no bound)
              distance_travelled: [null, 2.0]
    """
    return load_test_suite(yaml_path).cases
