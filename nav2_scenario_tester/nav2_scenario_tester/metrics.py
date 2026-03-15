"""Live metrics collection during navigation tests."""

from abc import ABC, abstractmethod
import math


class MetricsCollector(ABC):
    """
    Base class for live metrics collection.

    Subclasses declare which topics they need, receive messages live,
    and can optionally enforce limits that trigger early goal cancellation.
    """

    @abstractmethod
    def topics(self) -> list:
        """Return list of ROS topics this collector subscribes to."""

    @abstractmethod
    def on_message(self, topic: str, msg) -> None:
        """Process an incoming message (called from subscription callback)."""

    def check(self, limits: dict) -> str:
        """
        Check live data against limits.

        Args:
            limits: Dict of ``{metric_name: [min, max]}`` from the test case.

        Returns
        -------
        str or None
            Error description if a limit is breached, ``None`` otherwise.

        """
        return None

    @abstractmethod
    def report(self) -> dict:
        """Return final metrics dictionary after navigation completes."""

    def reset(self) -> None:
        """Clear buffered data between test runs."""


class OdometryMetrics(MetricsCollector):
    """Live odometry metrics: distance, speed, duration."""

    def __init__(self, odom_topic: str = '/odom'):
        self._odom_topic = odom_topic
        self._positions = []
        self._speeds = []
        self._timestamps = []
        self._distance = 0.0

    def topics(self) -> list:
        """Return required topics."""
        return [self._odom_topic]

    def on_message(self, topic: str, msg) -> None:
        """Buffer position and speed from odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._positions:
            dx = x - self._positions[-1][0]
            dy = y - self._positions[-1][1]
            self._distance += math.hypot(dx, dy)

        self._positions.append((x, y))
        self._timestamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._speeds.append(math.hypot(vx, vy))

    def check(self, limits: dict) -> str:
        """Check distance and speed against limits."""
        if 'distance_travelled' in limits:
            _, hi = limits['distance_travelled']
            if hi is not None and self._distance > hi:
                return (
                    f'distance_travelled={self._distance:.3f} exceeds max {hi}'
                )
        return None

    def report(self) -> dict:
        """Return final odometry metrics."""
        if len(self._positions) < 2:
            return {
                'distance_travelled': 0.0,
                'elapsed_time': 0.0,
                'avg_speed': 0.0,
                'max_speed': 0.0,
            }

        elapsed = self._timestamps[-1] - self._timestamps[0]
        return {
            'distance_travelled': self._distance,
            'elapsed_time': elapsed,
            'avg_speed': self._distance / elapsed if elapsed > 0 else 0.0,
            'max_speed': max(self._speeds) if self._speeds else 0.0,
        }

    def reset(self) -> None:
        """Clear buffered data."""
        self._positions.clear()
        self._speeds.clear()
        self._timestamps.clear()
        self._distance = 0.0


class PlanMetrics(MetricsCollector):
    """Live planning metrics from published plan paths."""

    def __init__(self, plan_topic: str = '/plan'):
        self._plan_topic = plan_topic
        self._plan_lengths = []

    def topics(self) -> list:
        """Return required topics."""
        return [self._plan_topic]

    def on_message(self, topic: str, msg) -> None:
        """Buffer plan length from each published path."""
        poses = msg.poses
        if len(poses) < 2:
            return

        length = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
            dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
            length += math.hypot(dx, dy)

        self._plan_lengths.append((length, len(poses)))

    def check(self, limits: dict) -> str:
        """Check plan length against limits."""
        if not self._plan_lengths:
            return None
        last_length = self._plan_lengths[-1][0]
        if 'plan_length' in limits:
            lo, hi = limits['plan_length']
            if lo is not None and last_length < lo:
                return f'plan_length={last_length:.3f} below min {lo}'
            if hi is not None and last_length > hi:
                return f'plan_length={last_length:.3f} exceeds max {hi}'
        return None

    def report(self) -> dict:
        """Return final planning metrics."""
        if not self._plan_lengths:
            return {
                'plan_length': 0.0,
                'plan_waypoints': 0,
                'plan_count': 0,
            }
        last_length, last_waypoints = self._plan_lengths[-1]
        return {
            'plan_length': last_length,
            'plan_waypoints': last_waypoints,
            'plan_count': len(self._plan_lengths),
        }

    def reset(self) -> None:
        """Clear buffered data."""
        self._plan_lengths.clear()
