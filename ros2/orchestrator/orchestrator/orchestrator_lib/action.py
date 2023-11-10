# pyright: strict

from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional, Union
from typing_extensions import TypeAlias
from orchestrator.orchestrator_lib.node_model import TopicInput, TimerInput
from rclpy.time import Time


class ActionState(Enum):
    WAITING = 1
    """Waiting for external input (message data or time increment)"""

    READY = 2
    """Data is buffered, ready to execute once allowed by ordering constraints"""

    RUNNING = 3
    """Running, expecting output as specified by model"""


class ActionNotFoundError(Exception):
    pass


@dataclass
class _BaseAction:
    """Base class for callback actions."""

    state: ActionState
    node: str
    timestamp: Time


@dataclass
class RxAction(_BaseAction):
    """Execution of a subscription callback at a ROS node."""

    cause: TopicInput
    """Description of causing topic input"""

    data: Optional[Any] = None
    """Cached input ROS message triggering this callback (deserialized)"""

    is_approximate_time_synced: bool = False
    """Flag indicating that this topic input belongs to an approximate-time-synced message filter.
    This implies that the filter-callback might not be executed for this input.
    """

    @property
    def topic(self) -> str:
        """Alias for cause.input_topic, provided for compatibility."""
        return self.cause.input_topic

    def __str__(self) -> str:
        return f"RxAction(state={self.state}, node={self.node}, timestamp={self.timestamp}, cause={self.cause}, " \
               f"data={'present' if self.data is not None else None}, " \
               f"approx_time_sync={self.is_approximate_time_synced})"


@dataclass
class TimerCallbackAction(_BaseAction):
    """Execution of a timer callback at a ROS node."""

    cause: TimerInput
    """Description of the causing input"""

    @property
    def period(self) -> int:
        """Alias for cause.period, provided for compatibility."""
        return self.cause.period


@dataclass
class DataProviderInputAction:
    state: ActionState
    published_topic: str


@dataclass
class OrchestratorBufferAction:
    """
    Dummy action to enable waiting for an output without actually triggering any other actions.
    Used during timer-initialization, for initial timer callbacks.
    """
    cause: TopicInput
    pass


@dataclass
class OrchestratorStatusAction:
    pass


Action: TypeAlias = Union[
    TimerCallbackAction, RxAction, OrchestratorBufferAction, OrchestratorStatusAction, DataProviderInputAction]

CallbackAction: TypeAlias = Union[TimerCallbackAction, RxAction]
OrchestratorAction: TypeAlias = Union[OrchestratorBufferAction,
OrchestratorStatusAction, DataProviderInputAction]


class EdgeType(Enum):
    CAUSALITY = 0
    """Edge points to the action which produces a required input"""

    SAME_NODE = 1
    """Edge points to a previous action at the same node"""

    SAME_TOPIC = 2
    """Points to a previous action receiving a topic published by this action"""

    SERVICE_GROUP = 3
    """Points to a previous action calling a service provided by this node, or to the provider of a service called by this action"""
