r"""
NavTestRunner -- reusable test runner for Nav2 BT + planner + controller integration tests.

Designed for persistent simulation: create one runner in setUpClass, call run()
with different start/goal poses per test method, and shut down in tearDownClass.

Usage from a launch_testing test file::

    from nav2_navigation_test import NavTestRunner
    from nav2_navigation_test.test_runner import make_pose

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

Or as a CLI tool::

    ros2 run nav2_navigation_test test_runner \\
        --start-x -2.0 --start-y -0.5 \\
        --goal-x 2.0 --goal-y 0.5 \\
        --timeout 60
"""

import argparse
from dataclasses import dataclass
import math
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass
class NavTestResult:
    """Result of a navigation test run."""

    success: bool
    elapsed_time: float
    error_code: int = 0
    error_msg: str = ''


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
    ):
        super().__init__(node_name=node_name, namespace=namespace)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10,
        )

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._stack_ready = False

    # ── Public API ─────────────────────────────────────────────────────

    def run(
        self,
        initial_pose: Pose,
        goal_pose: Pose,
        timeout: float = 60.0,
        settle_time: float = 2.0,
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

        Returns
        -------
        NavTestResult
            With success status and elapsed time.

        """
        # On first run, wait for the nav2 stack to be ready
        if not self._stack_ready:
            self.wait_for_node_active('bt_navigator')
            self._stack_ready = True

        # Teleport: publish initial pose and let the sim settle
        self.set_initial_pose(initial_pose)
        time.sleep(settle_time)

        start_time = time.time()

        # Send navigation goal via action
        nav_success, error_code, error_msg = self._navigate_to_pose(
            goal_pose, timeout,
        )

        elapsed = time.time() - start_time

        return NavTestResult(
            success=nav_success,
            elapsed_time=elapsed,
            error_code=error_code,
            error_msg=error_msg,
        )

    def set_initial_pose(self, pose: Pose) -> None:
        """Publish a pose to /initialpose (teleports the robot in loopback sim)."""
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = 'map'
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

    # ── Private helpers ────────────────────────────────────────────────

    def _navigate_to_pose(
        self, goal_pose: Pose, timeout: float,
    ) -> tuple[bool, int, str]:
        """Send NavigateToPose action and wait for result."""
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        if not self.action_client.wait_for_server(timeout_sec=timeout):
            return False, -1, 'NavigateToPose action server not available'

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose = goal_pose

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
        rclpy.spin_until_future_complete(self, get_result_future)

        status = get_result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            result = get_result_future.result().result
            return False, result.error_code, result.error_msg

        self.get_logger().info('Goal succeeded')
        return True, 0, ''


# ── CLI entry point ────────────────────────────────────────────────────


def make_pose(x: float, y: float, z: float = 0.01, yaw: float = 0.0) -> Pose:
    """Create a Pose message from x, y, z, yaw."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    # Convert yaw to quaternion (rotation around Z axis)
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main(argv: list[str] = sys.argv[1:]) -> int:
    parser = argparse.ArgumentParser(
        description='Nav2 navigation test runner CLI',
    )
    parser.add_argument('--start-x', type=float, default=-2.0)
    parser.add_argument('--start-y', type=float, default=-0.5)
    parser.add_argument('--start-yaw', type=float, default=0.0)
    parser.add_argument('--goal-x', type=float, default=0.0)
    parser.add_argument('--goal-y', type=float, default=2.0)
    parser.add_argument('--goal-yaw', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--namespace', type=str, default='')

    args, _ = parser.parse_known_args(argv)

    rclpy.init()

    runner = NavTestRunner(namespace=args.namespace)

    # Allow time for the full stack to come up
    time.sleep(10)

    result = runner.run(
        initial_pose=make_pose(args.start_x, args.start_y, yaw=args.start_yaw),
        goal_pose=make_pose(args.goal_x, args.goal_y, yaw=args.goal_yaw),
        timeout=args.timeout,
    )

    if result.success:
        runner.get_logger().info(
            f'TEST PASSED — reached goal in {result.elapsed_time:.1f}s'
        )
    else:
        runner.get_logger().error(
            f'TEST FAILED — '
            f'error_code: {result.error_code} error_msg: {result.error_msg}'
        )

    runner.shutdown()
    rclpy.shutdown()

    return 0 if result.success else 1


if __name__ == '__main__':
    sys.exit(main())
