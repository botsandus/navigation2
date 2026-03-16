"""Base class for live metrics collection."""

from abc import ABC, abstractmethod
from typing import Optional


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

    def check(self, limits: dict) -> Optional[str]:
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

    def setup(self, node) -> None:
        """Set up resources that require a ROS node, e.g. service clients."""

    @abstractmethod
    def report(self) -> dict:
        """Return final metrics dictionary after navigation completes."""

    def reset(self) -> None:
        """Clear buffered data between test runs."""
