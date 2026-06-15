"""
Minimal behaviour tree primitives for the deliberative planner.

Supports Selector (fallback), Sequence, Condition, and Action nodes.
"""

from enum import Enum, auto
from typing import Callable, List, Optional


class Status(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()


class Blackboard:
    """Shared planner state passed to every tree tick."""

    def __init__(self):
        self.safety_active = False
        self.front_range = float('inf')
        self.hello_streak = 0
        self.hello_cooldown_until = 0.0
        self.now = 0.0
        self.greeting_min = 1.8
        self.greeting_dist = 2.3
        self.safety_clear_dist = 1.2
        self.hello_confirm_ticks = 2
        self.hello_cooldown_s = 12.0
        self.chosen_behaviour: Optional[str] = None
        self.chosen_reason: Optional[str] = None


class Node:
    name: str

    def tick(self, blackboard: Blackboard) -> Status:
        raise NotImplementedError


class Sequence(Node):
    def __init__(self, name: str, children: List[Node]):
        self.name = name
        self.children = children

    def tick(self, blackboard: Blackboard) -> Status:
        for child in self.children:
            status = child.tick(blackboard)
            if status != Status.SUCCESS:
                return status
        return Status.SUCCESS


class Selector(Node):
    def __init__(self, name: str, children: List[Node]):
        self.name = name
        self.children = children

    def tick(self, blackboard: Blackboard) -> Status:
        for child in self.children:
            status = child.tick(blackboard)
            if status == Status.SUCCESS:
                return Status.SUCCESS
            if status == Status.RUNNING:
                return Status.RUNNING
        return Status.FAILURE


class Condition(Node):
    def __init__(self, name: str, predicate: Callable[[Blackboard], bool]):
        self.name = name
        self.predicate = predicate

    def tick(self, blackboard: Blackboard) -> Status:
        if self.predicate(blackboard):
            return Status.SUCCESS
        return Status.FAILURE


class Action(Node):
    def __init__(self, name: str, action: Callable[[Blackboard], Status]):
        self.name = name
        self.action = action

    def tick(self, blackboard: Blackboard) -> Status:
        return self.action(blackboard)
