r"""
NavTestRunner — reusable test runner for Nav2 BT + planner + controller integration tests.

Usage from a test file:

    from nav2_navigation_test import NavTestRunner
    from geometry_msgs.msg import Pose

    runner = NavTestRunner(
        initial_pose=Pose(...),
        goal_pose=Pose(...),
    )
    result = runner.run()
    assert result.success
    runner.shutdown()

Or as a CLI tool:

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
from typing import Optional

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
    final_pose: Optional[Pose]
    distance_from_goal: float
    error_code: int = 0
    error_msg: str = ''


class NavTestRunner(Node):
    """
    Reusable navigation test runner.

    Manages the full lifecycle of a navigation test: setting initial pose,
    sending goals via the NavigateToPose action, monitoring progress, and
    verifying the robot reached the goal pose.
    """

    def __init__(
        self,
        initial_pose: Pose,
        goal_pose: Pose,
        namespace: str = '',
        node_name: str = 'nav_test_runner',
    ):
        super().__init__(node_name=node_name, namespace=namespace)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10,
        )

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.initial_pose = initial_pose
        self.goal_pose = goal_pose
        self.current_pose = initial_pose

    # ── Public API ─────────────────────────────────────────────────────

    def run(
        self,
        timeout: float = 60.0,
        goal_distance_tolerance: float = 0.5,
    ) -> NavTestResult:
        """
        Execute the full navigation test: set initial pose, navigate, check result.

        Args:
            timeout: Maximum seconds to wait for navigation to complete.
            goal_distance_tolerance: Distance (m) within which the goal is considered reached.

        Returns
        -------
        NavTestResult
            Success status, elapsed time, and final pose info.

        """
        start_time = time.time()

        # Publish initial pose (loopback sim needs it to start processing
        # cmd_vel; AMCL uses it for localisation)
        self.set_initial_pose()

        # Wait for bt_navigator to be active
        self.wait_for_node_active('bt_navigator')

        # Send navigation goal via action
        nav_success, error_code, error_msg = self._navigate_to_pose(timeout)

        elapsed = time.time() - start_time
        dist = self._distance_from_goal()

        return NavTestResult(
            success=nav_success and dist < goal_distance_tolerance,
            elapsed_time=elapsed,
            final_pose=self.current_pose,
            distance_from_goal=dist,
            error_code=error_code,
            error_msg=error_msg,
        )

    def set_initial_pose(self) -> None:
        """Publish the initial pose to the /initialpose topic."""
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        self.get_logger().info('Publishing initial pose')
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

    def _navigate_to_pose(self, timeout: float) -> tuple[bool, int, str]:
        """Send NavigateToPose action and wait for result."""
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        if not self.action_client.wait_for_server(timeout_sec=timeout):
            return False, -1, 'NavigateToPose action server not available'

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose = self.goal_pose

        self.get_logger().info('Sending navigation goal')
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

    def _distance_from_goal(self) -> float:
        dx = self.current_pose.position.x - self.goal_pose.position.x
        dy = self.current_pose.position.y - self.goal_pose.position.y
        return math.sqrt(dx * dx + dy * dy)


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
    parser.add_argument('--tolerance', type=float, default=0.5)
    parser.add_argument('--namespace', type=str, default='')

    args, _ = parser.parse_known_args(argv)

    rclpy.init()

    runner = NavTestRunner(
        initial_pose=make_pose(args.start_x, args.start_y, yaw=args.start_yaw),
        goal_pose=make_pose(args.goal_x, args.goal_y, yaw=args.goal_yaw),
        namespace=args.namespace,
    )

    # Allow time for the full stack to come up
    time.sleep(10)

    result = runner.run(
        timeout=args.timeout,
        goal_distance_tolerance=args.tolerance,
    )

    if result.success:
        runner.get_logger().info(
            f'TEST PASSED — reached goal in {result.elapsed_time:.1f}s '
            f'(distance: {result.distance_from_goal:.3f}m)'
        )
    else:
        runner.get_logger().error(
            f'TEST FAILED — distance from goal: {result.distance_from_goal:.3f}m '
            f'error_code: {result.error_code} error_msg: {result.error_msg}'
        )

    runner.shutdown()
    rclpy.shutdown()

    return 0 if result.success else 1


if __name__ == '__main__':
    sys.exit(main())
