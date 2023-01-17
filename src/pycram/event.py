from typing import Callable, List, Optional, Any

class Event:

    def __init__(self):
        self.handlers: List[Callable] = []

    def add(self, handler: Callable) -> None:
        self.handlers.append(handler)

    def remove(self, handler: Callable) -> None:
        self.handlers.remove(handler)

    def fire(self, sender: Any, earg: Optional[Any] = None) -> None:
        for handler in self.handlers:
            handler(sender, earg)

    def __iadd__(self, other: Callable):
        self.add(other)
        return self

    def __isub__(self, other: Callable):
        self.remove(other)
        return self

    __call__ = fire
