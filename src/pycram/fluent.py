# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import operator

from enum import Enum
from threading import Condition, Lock
from uuid import uuid4
from typing_extensions import Any, Optional, List, Callable


class Behavior(Enum):
    """Enumeration which can be passed as argument to the pulsed method of fluents to describe how to handle missed pulses in the whenever macro.

    Fields:
    NEVER -- ignore missed pulses.
    ONCE -- if pulses were missed, execute the body once more in total.
    ALWAYS -- if pulses were missed, execute the body once more for each.
    """

    NEVER = 1
    ONCE = 2
    ALWAYS = 3


class Fluent:
    """Implementation of fluents.

    Fluents are thread-safe proxy objects which are used as variables with changing value. This allows threads to
    observe them and wait for (specific) changes.
    One can also observe fluents created by the pulsed method of a fluent. These change their value from None to True
    whenever the parent gets pulsed (or changes its value and thus gets pulsed).

    Fluents can be combined to fluent networks which allows to express complex conditions. A network updates its value
    whenever one of the fluents it is constructed from changes its value.
    The most important comparison and math operators (<, <=, ==, !=, >, >=, +, -, *, /) are overloaded to construct a
    fluent network whenever they are called with at least one fluent as parameter. In addition the comparison operators
    IS and IS_NOT as well as the logical operators AND, OR and NOT are defined as methods. This is necessary because
    these operators can't be overloaded. Fluents constructed by comparison or logical operators have the value True or
    None instead of False, so that they can be used with the wait_for method because it blocks until a value is not None.
    User defined operators can be created by passing a function as the fluents value.
    """

    def __init__(self, value: Optional[Any] = None, name: str = None):
        """Create a new fluent.

        :param value:  the value of the fluent which can also be a function to create user defined operators (default is None).
        :param name: the name of the fluent (default is a random string).
        """
        self._cv: Condition = Condition()
        self._mutex: Lock = Lock()
        self._pulses: int = 0
        self._children: List[Fluent] = []
        self._handle_missed = Behavior.NEVER
        self._value: Any = value
        self._whenever_cbs = []

        if name is not None:
            self.name: str = name
        else:
            self.name: str = str(uuid4())

    def pulsed(self, handle_missed: Behavior = 2) -> Fluent:
        """Create a fluent which changes its value from None to True whenever the parent gets pulsed.

        :param handle_missed: see the docstring of the Behavior enumeration to find out more (default is Behavior.ONCE).
        """
        fluent = Fluent()

        def value():
            if fluent._pulses != 0:
                return True
            else:
                return None

        fluent.set_value(value)
        fluent._handle_missed = handle_missed
        self.add_child(fluent)
        return fluent

    def pulse(self) -> None:
        """Pulse a fluent without changing its value."""
        for child in self._children:
            with child._mutex:
                child._pulses += 1

            child.pulse()
        for callback in self._whenever_cbs:
            callback(self.get_value())
            with self._mutex:
                self._pulses -= 1
            if self._handle_missed == Behavior.NEVER:
                with self._mutex:
                    self._pulses = 0
            elif self._handle_missed == Behavior.ONCE:
                callback(self.get_value())
                with self._mutex:
                    self._pulses = 0
            elif self._handle_missed == Behavior.ALWAYS:
                with self._mutex:
                    for i in range(self._pulses):
                        callback(self.get_value())
        with self._cv:
            self._cv.notify()

    def whenever(self, callback: Callable) -> None:
        """
        Registers a callback which is called everytime this Fluent is pulsed. The callback should be a Callable. When
        the callback is called it gets the current value of this Fluent as an argument.

        :param callback: The callback which should be called when pulsed as a Callable.
        """
        self._whenever_cbs.append(callback)

    def add_child(self, child: Fluent) -> None:
        """Add a child to the fluent which gets pulsed whenever this fluent gets pulsed, too.

        :param child: the child to add.
        """
        self._children.append(child)

    def get_value(self) -> Any:
        """Return the value of the fluent."""
        with self._mutex:
            if callable(self._value):
                return self._value()

            return self._value

    def set_value(self, value: Any) -> None:
        """Change the value of the fluent.
        Changing the value will also pulse the fluent.

        :param value: the new value of the fluent.
        """
        with self._mutex:
            self._value = value

        self.pulse()

    def wait_for(self, timeout: Optional[float] = None):
        """Block the current thread if the value of the fluent is None, until it is not None or until it timed out.

        If the fluent was created by the pulsed method of a fluent, the method blocks until the parent gets pulsed.

        The return value is the last return value of the predicate (value is not None) and will evaluate to False if the method timed out.

        :param timeout: the maximum time to wait (default is None).
        """
        with self._cv:
            return self._cv.wait_for(lambda: self.get_value() is not None, timeout)

    def _compare(self, operator, other: Fluent) -> Fluent:
        """This is a helper method for internal usage only.

        Create a fluent which value is a function returning True or None depending on the given comparison operator applied to the operands.

        :param operator: the comparison operator to apply.
        :param other: the other operand.
        """
        fluent = Fluent()

        if type(other) == Fluent:

            def value():
                if operator(self.get_value(), other.get_value()):
                    return True
                else:
                    return None

            other.add_child(fluent)
        else:

            def value():
                if operator(self.get_value(), other):
                    return True
                else:
                    return None

        self.add_child(fluent)
        fluent.set_value(value)
        return fluent

    def __lt__(self, other: Fluent) -> Fluent:
        """Overload the < comparison operator.

        :param other: -- the other operand.
        """
        return self._compare(operator.lt, other)

    def __leq__(self, other: Fluent) -> Fluent:
        """Overload the <= comparison operator.

        :param other: the other operand.
        """
        return self._compare(operator.leq, other)

    def __eq__(self, other: Fluent) -> Fluent:
        """Overload the == comparison operator.

        :param other: the other operand.
        """
        return self._compare(operator.eq, other)

    def __ne__(self, other: Fluent) -> Fluent:
        """Overload the != comparison operator.

        :param other: the other operand.
        """
        return self._compare(operator.ne, other)

    def IS(self, other: Fluent) -> Fluent:
        """Create a fluent which value is True if the value of its parent is the value of the given operand, None otherwise.

        :param other: -- the other operand which can also be a fluent.
        """
        return self._compare(operator.is_, other)

    def IS_NOT(self, other: Fluent) -> Fluent:
        """Create a fluent which value is True if the value of its parent is not the value of the given operand, None otherwise.

        :param other: -- the other operand which can also be a fluent.
        """
        return self._compare(operator.is_not, other)

    def __gt__(self, other: Fluent) -> Fluent:
        """Overload the > comparison operator.

        :param other: the other operand.
        """
        return self._compare(operator.gt, other)

    def __geq__(self, other: Fluent) -> Fluent:
        """Overload the >= comparison operator.

        :param other: the other operand.
        """
        return self._compare(operator.geq, other)

    def _math(self, operator: Callable, operand: Fluent, other: Fluent) -> Fluent:
        """This is a helper method for internal usage only.

        Create a fluent which value is a function returning the value of the given math operator applied to the operands.

        :param operator: the math operator to apply.
        :param operand: the first operand.
        :param other: the other operand.
        """
        fluent = Fluent()

        if type(operand) == Fluent:
            if type(other) == Fluent:
                value = lambda: operator(operand.get_value(), other.get_value())
                other.add_child(fluent)
            else:
                value = lambda: operator(operand.get_value(), other)

            operand.add_child(fluent)
        else:
            value = lambda: operator(operand, other.get_value())
            other.add_child(fluent)

        fluent.set_value(value)
        return fluent

    def __add__(self, other: Fluent) -> Fluent:
        """Overload the + math operator.

        :parm other: the other operand.
        """
        return self._math(operator.add, self, other)

    def __radd__(self, other: Fluent) -> Fluent:
        """Overload the + math operator with the first operand not being a fluent.

        :param other: the other operand.
        """
        return self._math(operator.add, other, self)

    def __sub__(self, other: Fluent) -> Fluent:
        """Overload the - math operator.

        :param other: the other operand.
        """
        return self._math(operator.sub, self, other)

    def __rsub__(self, other: Fluent) -> Fluent:
        """Overload the - math operator with the first operand not being a fluent.

        :param other: the other operand.
        """
        return self._math(operator.sub, other, self)

    def __mul__(self, other: Fluent) -> Fluent:
        """Overload the * math operator.

        :param other: the other operand.
        """
        return self._math(operator.mul, self, other)

    def __rmul__(self, other: Fluent) -> Fluent:
        """Overload the * math operator with the first operand not being a fluent.

        :param other: the other operand.
        """
        return self._math(operator.mul, other, self)

    def __truediv__(self, other: Fluent) -> Fluent:
        """Overload the / math operator.

        :param other: the other operand.
        """
        return self._math(operator.truediv, self, other)

    def __rtruediv__(self, other) -> Fluent:
        """Overload the / math operator with the first operand not being a fluent.

        :param other: the other operand.
        """
        return self._math(operator.truediv, other, self)

    def AND(self, other: Fluent) -> Fluent:
        """Create a fluent which value is True if both, the value of its parent and the other operand express True, None otherwise.

        :param other: the other operand which can also be a fluent.
        """
        fluent = Fluent()

        if type(other) == Fluent:

            def value():
                if self.get_value() and other.get_value():
                    return True
                else:
                    return None

            other.add_child(fluent)
        else:

            def value():
                if self.get_value() and other:
                    return True
                else:
                    return None

        self.add_child(fluent)
        fluent.set_value(value)
        return fluent

    def OR(self, other: Fluent) -> Fluent:
        """Create a fluent which value is True if either the value of its parent or the other operand express True, None otherwise.

        :param other: the other operand which can also be a fluent.
        """
        fluent = Fluent()

        if type(other) == Fluent:

            def value():
                if self.get_value() or other.get_value():
                    return True
                else:
                    return None

            other.add_child(fluent)
        else:

            def value():
                if self.get_value() or other:
                    return True
                else:
                    return None

        self.add_child(fluent)
        fluent.set_value(value)
        return fluent

    def NOT(self) -> Fluent:
        """Create a fluent which value is True if the value of its parent expresses False, None otherwise."""

        def value():
            if not self.get_value():
                return True
            else:
                return None

        fluent = Fluent(value)
        self.add_child(fluent)
        return fluent
