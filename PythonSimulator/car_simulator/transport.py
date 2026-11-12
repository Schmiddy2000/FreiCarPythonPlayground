"""Synchronous local publishers replacing the ROS communication layer."""
from collections.abc import Callable
from copy import deepcopy
from typing import Generic, TypeVar

MessageType = TypeVar("MessageType")


class Publisher(Generic[MessageType]):
    def __init__(self, transient_local: bool = False):
        self.transient_local = transient_local
        self.subscribers: list[Callable[[MessageType], None]] = []
        self.last_message: MessageType | None = None

    def subscribe(self, callback: Callable[[MessageType], None]) -> None:
        self.subscribers.append(callback)
        if self.transient_local and self.last_message is not None:
            callback(deepcopy(self.last_message))

    def publish(self, message: MessageType) -> None:
        # Deliver values rather than references to a mutable controller command.
        self.last_message = deepcopy(message)
        for callback in self.subscribers:
            callback(deepcopy(message))
