"""Live odometry metrics: distance, speed, duration."""

from collections import deque
import math

from nav2_scenario_tester.metrics._base import MetricsCollector


class OdometryMetrics(MetricsCollector):
    """Live odometry metrics: distance, speed, duration."""

    _ACCEL_WINDOW = 5  # sliding window size for smoothing acceleration

    def __init__(self, odom_topic: str = '/odom'):
        self._odom_topic = odom_topic
        self._positions = []
        self._speeds = []
        self._angular_speeds = []
        self._accelerations = []
        self._decelerations = []
        self._angular_accelerations = []
        self._angular_decelerations = []
        self._timestamps = []
        self._distance = 0.0
        self._raw_lin_accel = deque(maxlen=self._ACCEL_WINDOW)
        self._raw_lin_decel = deque(maxlen=self._ACCEL_WINDOW)
        self._raw_ang_accel = deque(maxlen=self._ACCEL_WINDOW)
        self._raw_ang_decel = deque(maxlen=self._ACCEL_WINDOW)

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

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        linear_speed = abs(msg.twist.twist.linear.x)
        angular_speed = abs(msg.twist.twist.angular.z)

        if self._timestamps:
            dt = stamp - self._timestamps[-1]
            if dt > 0:
                lin_delta = (linear_speed - self._speeds[-1]) / dt
                ang_delta = (angular_speed - self._angular_speeds[-1]) / dt

                if lin_delta >= 0:
                    self._raw_lin_accel.append(lin_delta)
                    self._accelerations.append(
                        sum(self._raw_lin_accel) / len(self._raw_lin_accel)
                    )
                else:
                    self._raw_lin_decel.append(-lin_delta)
                    self._decelerations.append(
                        sum(self._raw_lin_decel) / len(self._raw_lin_decel)
                    )

                if ang_delta >= 0:
                    self._raw_ang_accel.append(ang_delta)
                    self._angular_accelerations.append(
                        sum(self._raw_ang_accel) / len(self._raw_ang_accel)
                    )
                else:
                    self._raw_ang_decel.append(-ang_delta)
                    self._angular_decelerations.append(
                        sum(self._raw_ang_decel) / len(self._raw_ang_decel)
                    )

        self._timestamps.append(stamp)
        self._speeds.append(linear_speed)
        self._angular_speeds.append(angular_speed)

    def check(self, limits: dict) -> str:
        """Check distance, speed, and acceleration against limits."""
        if 'distance_travelled' in limits:
            lo, hi = limits['distance_travelled']
            if hi is not None and self._distance > hi:
                return (
                    f'distance_travelled={self._distance:.3f} exceeds max {hi}'
                )
            if lo is not None and self._distance < lo:
                pass  # lower bound only meaningful at end, not during navigation
        if 'linear_speed' in limits and self._speeds:
            lo, hi = limits['linear_speed']
            peak = max(self._speeds)
            if hi is not None and peak > hi:
                return f'linear_speed={peak:.3f} exceeds max {hi}'
        if 'angular_speed' in limits and self._angular_speeds:
            lo, hi = limits['angular_speed']
            peak = max(self._angular_speeds)
            if hi is not None and peak > hi:
                return f'angular_speed={peak:.3f} exceeds max {hi}'
        if 'linear_acceleration' in limits and self._accelerations:
            lo, hi = limits['linear_acceleration']
            peak = max(self._accelerations)
            if hi is not None and peak > hi:
                return f'linear_acceleration={peak:.3f} exceeds max {hi}'
        if 'linear_deceleration' in limits and self._decelerations:
            lo, hi = limits['linear_deceleration']
            peak = max(self._decelerations)
            if hi is not None and peak > hi:
                return f'linear_deceleration={peak:.3f} exceeds max {hi}'
        if 'angular_acceleration' in limits and self._angular_accelerations:
            lo, hi = limits['angular_acceleration']
            peak = max(self._angular_accelerations)
            if hi is not None and peak > hi:
                return f'angular_acceleration={peak:.3f} exceeds max {hi}'
        if 'angular_deceleration' in limits and self._angular_decelerations:
            lo, hi = limits['angular_deceleration']
            peak = max(self._angular_decelerations)
            if hi is not None and peak > hi:
                return f'angular_deceleration={peak:.3f} exceeds max {hi}'
        return None

    def report(self) -> dict:
        """Return final odometry metrics."""
        if len(self._positions) < 2:
            return {
                'distance_travelled': 0.0,
                'elapsed_time': 0.0,
                'linear_speed': 0.0,
                'angular_speed': 0.0,
                'linear_acceleration': 0.0,
                'linear_deceleration': 0.0,
                'angular_acceleration': 0.0,
                'angular_deceleration': 0.0,
            }

        elapsed = self._timestamps[-1] - self._timestamps[0]
        return {
            'distance_travelled': self._distance,
            'elapsed_time': elapsed,
            'linear_speed': max(self._speeds) if self._speeds else 0.0,
            'angular_speed': max(self._angular_speeds) if self._angular_speeds else 0.0,
            'linear_acceleration': max(self._accelerations) if self._accelerations else 0.0,
            'linear_deceleration': max(self._decelerations) if self._decelerations else 0.0,
            'angular_acceleration': (
                max(self._angular_accelerations)
                if self._angular_accelerations else 0.0
            ),
            'angular_deceleration': (
                max(self._angular_decelerations)
                if self._angular_decelerations else 0.0
            ),
        }

    def reset(self) -> None:
        """Clear buffered data."""
        self._positions.clear()
        self._speeds.clear()
        self._angular_speeds.clear()
        self._accelerations.clear()
        self._decelerations.clear()
        self._angular_accelerations.clear()
        self._angular_decelerations.clear()
        self._timestamps.clear()
        self._distance = 0.0
        self._raw_lin_accel.clear()
        self._raw_lin_decel.clear()
        self._raw_ang_accel.clear()
        self._raw_ang_decel.clear()
