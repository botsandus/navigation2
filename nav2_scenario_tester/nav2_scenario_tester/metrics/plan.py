"""Live planning metrics from published plan paths."""

import math

from nav2_scenario_tester.metrics._base import MetricsCollector


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
