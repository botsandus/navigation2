"""Live tracking error metrics from controller feedback."""

from nav2_scenario_tester.metrics._base import MetricsCollector


class TrackingMetrics(MetricsCollector):
    """Track position and heading errors from controller feedback."""

    def __init__(self, topic: str = '/tracking_feedback'):
        self._topic = topic
        self._position_errors = []
        self._heading_errors = []

    def topics(self) -> list:
        """Return required topics."""
        return [self._topic]

    def on_message(self, topic: str, msg) -> None:
        """Buffer tracking errors from each feedback message."""
        self._position_errors.append(abs(msg.position_tracking_error))
        self._heading_errors.append(abs(msg.heading_tracking_error))

    def check(self, limits: dict) -> str:
        """Check tracking errors against limits."""
        if 'position_tracking_error' in limits and self._position_errors:
            lo, hi = limits['position_tracking_error']
            peak = max(self._position_errors)
            if hi is not None and peak > hi:
                return (
                    f'position_tracking_error={peak:.3f}'
                    f' exceeds max {hi}'
                )
        if 'heading_tracking_error' in limits and self._heading_errors:
            lo, hi = limits['heading_tracking_error']
            peak = max(self._heading_errors)
            if hi is not None and peak > hi:
                return (
                    f'heading_tracking_error={peak:.3f}'
                    f' exceeds max {hi}'
                )
        return None

    def report(self) -> dict:
        """Return tracking error metrics as ``(min, max)`` tuples."""
        if not self._position_errors:
            return {
                'position_tracking_error': (0.0, 0.0),
                'heading_tracking_error': (0.0, 0.0),
            }
        return {
            'position_tracking_error': (
                min(self._position_errors),
                max(self._position_errors),
            ),
            'heading_tracking_error': (
                min(self._heading_errors),
                max(self._heading_errors),
            ),
        }

    def reset(self) -> None:
        """Clear buffered data."""
        self._position_errors.clear()
        self._heading_errors.clear()
