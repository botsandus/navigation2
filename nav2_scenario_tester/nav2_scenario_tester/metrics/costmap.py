"""Track max footprint cost via the GetCosts service during navigation."""

import logging

from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import GetCosts
from nav2_scenario_tester.metrics._base import MetricsCollector

logger = logging.getLogger(__name__)


class CostmapMetrics(MetricsCollector):
    """Track max footprint cost via the GetCosts service."""

    def __init__(
        self,
        service_name: str = '/local_costmap/get_cost_local_costmap',
        poll_period: float = 0.5,
    ):
        self._service_name = service_name
        self._poll_period = poll_period
        self._node = None
        self._client = None
        self._timer = None
        self._max_footprint_cost = 0.0
        self._min_footprint_cost = float('inf')
        self._max_center_cost = 0.0
        self._min_center_cost = float('inf')
        self._has_footprint = False
        self._has_center = False

    def topics(self) -> list:
        """No topic subscriptions needed — uses a service."""
        return []

    def setup(self, node) -> None:
        """Create the GetCosts service client and polling timer."""
        self._node = node
        self._client = node.create_client(GetCosts, self._service_name)
        self._timer = node.create_timer(self._poll_period, self._poll_cost)

    def on_message(self, topic: str, msg) -> None:
        """Not used — cost is polled via service."""

    def _poll_cost(self) -> None:
        """Timer callback: send async GetCosts requests for the robot pose."""
        if self._client is None or not self._client.service_is_ready():
            return

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.orientation.w = 1.0

        footprint_req = GetCosts.Request()
        footprint_req.use_footprint = True
        footprint_req.poses = [pose]
        self._client.call_async(footprint_req).add_done_callback(
            self._on_footprint_response
        )

        center_req = GetCosts.Request()
        center_req.use_footprint = False
        center_req.poses = [pose]
        self._client.call_async(center_req).add_done_callback(
            self._on_center_response
        )

    def _on_footprint_response(self, future) -> None:
        """Process the footprint GetCosts response."""
        try:
            result = future.result()
        except Exception:
            logger.debug('GetCosts service call failed', exc_info=True)
            return
        if not result.success or not result.costs:
            return
        self._max_footprint_cost = max(self._max_footprint_cost, result.costs[0])
        self._min_footprint_cost = min(self._min_footprint_cost, result.costs[0])
        self._has_footprint = True

    def _on_center_response(self, future) -> None:
        """Process the center-point GetCosts response."""
        try:
            result = future.result()
        except Exception:
            logger.debug('GetCosts service call failed', exc_info=True)
            return
        if not result.success or not result.costs:
            return
        self._max_center_cost = max(self._max_center_cost, result.costs[0])
        self._min_center_cost = min(self._min_center_cost, result.costs[0])
        self._has_center = True

    def check(self, limits: dict) -> str:
        """Check max footprint and center cost against limits."""
        if 'footprint_cost' in limits:
            lo, hi = limits['footprint_cost']
            if hi is not None and self._max_footprint_cost > hi:
                return (
                    f'footprint_cost={self._max_footprint_cost:.0f} exceeds max {hi}'
                )
            if lo is not None and self._max_footprint_cost < lo:
                return (
                    f'footprint_cost={self._max_footprint_cost:.0f} below min {lo}'
                )
        if 'center_cost' in limits:
            lo, hi = limits['center_cost']
            if hi is not None and self._max_center_cost > hi:
                return (
                    f'center_cost={self._max_center_cost:.0f} exceeds max {hi}'
                )
            if lo is not None and self._max_center_cost < lo:
                return (
                    f'center_cost={self._max_center_cost:.0f} below min {lo}'
                )
        return None

    def report(self) -> dict:
        """Return costmap proximity metrics as ``(min, max)`` tuples."""
        return {
            'footprint_cost': (
                self._min_footprint_cost if self._has_footprint else 0.0,
                self._max_footprint_cost,
            ),
            'center_cost': (
                self._min_center_cost if self._has_center else 0.0,
                self._max_center_cost,
            ),
        }

    def reset(self) -> None:
        """Clear buffered data and restart the polling timer."""
        self._max_footprint_cost = 0.0
        self._min_footprint_cost = float('inf')
        self._max_center_cost = 0.0
        self._min_center_cost = float('inf')
        self._has_footprint = False
        self._has_center = False
        if self._timer is not None:
            self._timer.reset()
