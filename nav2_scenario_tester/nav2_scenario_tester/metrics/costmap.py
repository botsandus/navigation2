"""Track max footprint cost via the GetCosts service during navigation."""

from nav2_scenario_tester.metrics._base import MetricsCollector


class CostmapMetrics(MetricsCollector):
    """Track max footprint cost via the GetCosts service during navigation."""

    def __init__(
        self,
        service_name: str = '/local_costmap/get_costs',
        poll_period: float = 0.5,
    ):
        self._service_name = service_name
        self._poll_period = poll_period
        self._node = None
        self._client = None
        self._timer = None
        self._max_cost = 0.0
        self._costs = []

    def topics(self) -> list:
        """No topic subscriptions needed — uses a service."""
        return []

    def setup(self, node) -> None:
        """Create the GetCosts service client and polling timer."""
        from nav2_msgs.srv import GetCosts
        self._node = node
        self._client = node.create_client(GetCosts, self._service_name)
        self._timer = node.create_timer(self._poll_period, self._poll_cost)

    def on_message(self, topic: str, msg) -> None:
        """Not used — cost is polled via service."""

    def _poll_cost(self) -> None:
        """Timer callback: send an async GetCosts request for the robot pose."""
        if self._client is None or not self._client.service_is_ready():
            return

        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.srv import GetCosts

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.orientation.w = 1.0

        req = GetCosts.Request()
        req.use_footprint = True
        req.poses = [pose]

        future = self._client.call_async(req)
        future.add_done_callback(self._on_cost_response)

    def _on_cost_response(self, future) -> None:
        """Process the GetCosts response."""
        try:
            result = future.result()
        except Exception:
            return
        if not result.success or not result.costs:
            return

        cost = result.costs[0]
        self._costs.append(cost)
        self._max_cost = max(self._max_cost, cost)

    def check(self, limits: dict) -> str:
        """Check max footprint cost against limits."""
        if 'max_footprint_cost' in limits:
            _, hi = limits['max_footprint_cost']
            if hi is not None and self._max_cost > hi:
                return (
                    f'max_footprint_cost={self._max_cost:.0f} exceeds max {hi}'
                )
        return None

    def report(self) -> dict:
        """Return costmap proximity metrics."""
        return {
            'max_footprint_cost': self._max_cost,
            'avg_footprint_cost': (
                sum(self._costs) / len(self._costs)
                if self._costs else 0.0
            ),
            'samples': len(self._costs),
        }

    def reset(self) -> None:
        """Clear buffered data."""
        self._max_cost = 0.0
        self._costs.clear()
