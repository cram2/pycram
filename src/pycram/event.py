# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from typing_extensions import Callable, List, Optional, Any


class Event:
    """
    Base implementation of events in PyCRAM.
    Events allow to attach handler methods to events that fire for specific occurences in the world.

    :ivar handler: List of methods that are called when this event is fired.
    """

    def __init__(self):
        self.handlers: List[Callable] = []

    def add(self, handler: Callable) -> None:
        """
        Adds a new handler to the list of handlers. All handler methods are called when this event is fired.
        Handler have to take the event sender as parameter as well as args* which can contain further parameter.

        :param handler: A method that should be added
        """
        self.handlers.append(handler)

    def remove(self, handler: Callable) -> None:
        """
        Removes a method from the list of handlers, the method will not be called when the event is fired.

        :param handler: The method that should be removed.
        """
        self.handlers.remove(handler)

    def fire(self, sender: Any, earg: Optional[Any] = None) -> None:
        """
        Fire this event, this causes every method to be called with a sender as well as additional args.

        :param sender: The entity that fired the event.
        :param earg: Additional arguments.
        """
        for handler in self.handlers:
            handler(sender, earg)

    def __iadd__(self, other: Callable) -> Event:
        """
        Operator overload that allows to add handlers by the '+=' operator.

        :param other: The handler that should be added.
        :return: This instance
        """
        self.add(other)
        return self

    def __isub__(self, other: Callable) -> Event:
        """
        Operator overload that allows to remove methods as handlers by using the '-=' operator.

        :param other: The method that should be removed as handler.
        :return: This instance
        """
        self.remove(other)
        return self

    __call__ = fire
    """
    Allows to directly call the reference.
    """