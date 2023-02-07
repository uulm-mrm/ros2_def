from dataclasses import dataclass
from enum import Enum
from typing import Any


class ActionState(Enum):
    WAITING = 1  # Waiting for data, no message buffered
    READY = 2  # Data is buffered, ready to execute once allowed by ordering constraints
    RUNNING = 3  # Running, expecting output as specified by model


class ActionNotFoundError(Exception):
    pass


@dataclass
class RxAction:
    state: ActionState
    node: str
    topic: str
    data: Any | None = None


class EdgeType(Enum):
    CAUSALITY = 0  # Edge points to the action which produces a required input
    SAME_NODE = 1  # Edge points to a previous action at the same node
    SAME_TOPIC = 2  # Points to a previous action receiving a topic published by this action
