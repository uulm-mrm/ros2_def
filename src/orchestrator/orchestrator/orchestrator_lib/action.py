from dataclasses import dataclass
from enum import Enum
from typing import Any
from orchestrator.orchestrator_lib.node_model import Cause
from rclpy.time import Time


class ActionState(Enum):
    WAITING = 1  # Waiting for external input (message data or time increment)
    READY = 2  # Data is buffered, ready to execute once allowed by ordering constraints
    RUNNING = 3  # Running, expecting output as specified by model


class ActionNotFoundError(Exception):
    pass


@dataclass
class Action:
    state: ActionState
    node: str
    timestamp: Time
    cause: Cause


@dataclass
class RxAction(Action):
    topic: str
    data: Any | None = None


@dataclass
class TimerCallbackAction(Action):
    period: int  # The period identifies the callback amongst possibly multiple timers with different period in a single node


class EdgeType(Enum):
    CAUSALITY = 0  # Edge points to the action which produces a required input
    SAME_NODE = 1  # Edge points to a previous action at the same node
    SAME_TOPIC = 2  # Points to a previous action receiving a topic published by this action
