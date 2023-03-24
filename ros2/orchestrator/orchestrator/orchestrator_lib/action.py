from dataclasses import dataclass
from enum import Enum
from typing import Any, TypeAlias
from orchestrator.orchestrator_lib.node_model import Cause, TopicInput, TimerInput
from rclpy.time import Time


class ActionState(Enum):
    WAITING = 1  # Waiting for external input (message data or time increment)
    READY = 2  # Data is buffered, ready to execute once allowed by ordering constraints
    RUNNING = 3  # Running, expecting output as specified by model


class ActionNotFoundError(Exception):
    pass


@dataclass
class _BaseAction:
    state: ActionState
    node: str
    timestamp: Time
    cause: Cause


@dataclass
class RxAction(_BaseAction):
    cause: TopicInput
    topic: str  # TODO: this is in cause
    data: Any | None = None
     # If this belongs to approximate time sync group, the resulting callback might
     # not be executed every time.
    is_approximate_time_synced: bool = False

@dataclass
class TimerCallbackAction(_BaseAction):
    cause: TimerInput
    # TODO: this actually is in the cause field...
    period: int  # The period identifies the callback amongst possibly multiple timers with different period in a single node


@dataclass
class DataProviderInputAction():
    state: ActionState
    published_topic: str


@dataclass
class OrchestratorBufferAction():
    """
    Dummy action to enable waiting for an output without actually triggering any other actions.
    Used during timer-initialization, for initial timer callbacks.
    """
    cause: TopicInput
    pass


@dataclass
class OrchestratorStatusAction():
    pass


Action: TypeAlias = TimerCallbackAction | RxAction | OrchestratorBufferAction | OrchestratorStatusAction | DataProviderInputAction

CallbackAction: TypeAlias = TimerCallbackAction | RxAction
OrchestratorAction: TypeAlias = OrchestratorBufferAction | OrchestratorStatusAction | DataProviderInputAction


class EdgeType(Enum):
    CAUSALITY = 0  # Edge points to the action which produces a required input
    SAME_NODE = 1  # Edge points to a previous action at the same node
    SAME_TOPIC = 2  # Points to a previous action receiving a topic published by this action
    SERVICE_GROUP = 3  # Points to a previous action calling a service provided by this node, or to the provider of a service called by this action
