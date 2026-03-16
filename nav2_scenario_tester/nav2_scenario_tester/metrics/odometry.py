"""Live odometry metrics: distance, speed, duration."""

import math

from nav2_scenario_tester.metrics._base import MetricsCollector


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
            lo, hi = limits['distance_travelled']
            if hi is not None and self._distance > hi:
                return (
                    f'distance_travelled={self._distance:.3f} exceeds max {hi}'
                )
            if lo is not None and self._distance < lo:
                pass  # lower bound only meaningful at end, not during navigation
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
