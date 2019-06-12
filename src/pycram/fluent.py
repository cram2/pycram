"""Implementation of fluents and the whenever macro.

Macros:
whenever -- macro to repeat a body as long as the value of a fluent is not None or whenever a pulsed fluent changes its value or gets pulsed.

Classes:
Behavior -- enumeration to describe how to handle missed pulses in the whenever macro.
Fluent -- implementation of fluents.
"""
from pycram.helper import _block
from enum import Enum
from macropy.core.macros import Macros
from macropy.core.hquotes import macros, hq
from macropy.core.quotes import macros, ast_literal
import operator
from threading import Condition, Lock
from uuid import uuid4

macros = Macros()
"""Must be imported before macros defined in this module can be imported."""

@macros.block
def whenever(tree, args, **kw):
	"""Execute the body as long as the value of the fluent passed as argument is not None. If the value is None the macro waits for it to become not None. A break statement is required to stop.

	If the passed fluent was created by the pulsed method of a fluent, the body gets executed whenever the parent gets pulsed.
	Missed pulses which occur while the body is executing are handled depending on the behavior passed as argument to the pulsed method.

	Arguments:
	fluent -- the fluent to watch the value of.

	Usage:
	with whenever(fluent):
		body
	"""
	with hq as new_tree:
		_fluent = ast_literal[args[0]]

		while True:
			_fluent.wait_for()
			ast_literal[tree]

			if _fluent._handle_missed == Behavior.NEVER:
				with _fluent._mutex:
					_fluent._pulses = 0 # Ignore missed pulses
			else:
				with _fluent._mutex:
					_fluent._pulses -= 1

					if _fluent._pulses > 1 and _fluent._handle_missed == Behavior.ONCE:
						_fluent._pulses = 1 # Execute body only once more

	return _block(new_tree)

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

	Fluents are thread-safe proxy objects which are used as variables with changing value. This allows threads to observe them and wait for (specific) changes.
	One can also observe fluents created by the pulsed method of a fluent. These change their value from None to True whenever the parent gets pulsed (or changes its value and thus gets pulsed).

	Fluents can be combined to fluent networks which allows to express complex conditions. A network updates its value whenever one of the fluents it is constructed from changes its value.
	The most important comparison and math operators (<, <=, ==, !=, >, >=, +, -, *, /) are overloaded to construct a fluent network whenever they are called with at least one fluent as parameter. In addition the comparison operators IS and IS_NOT as well as the logical operators AND, OR and NOT are defined as methods. This is necessary because these operators can't be overloaded. Fluents constructed by comparison or logical operators have the value True or None instead of False, so that they can be used with the wait_for method because it blocks until a value is not None.
	User defined operators can be created by passing a function as the fluents value.

	Instance variables:
	name -- the name of the fluent.

	Methods:
	pulsed -- create a fluent which changes its value from None to True whenever the parent gets pulsed.
	pulse -- pulse a fluent without changing its value.
	add_child -- add a child to the fluent.
	get_value -- return the value of the fluent.
	set_value -- change the value of the fluent.
	wait_for -- block the current thread if the value of the fluent is None, until it is not None or until it timed out.
	IS -- create a fluent which value is True if the value of its parent is the value of the given operand, None otherwise.
	IS_NOT -- create a fluent which value is True if the value of its parent is not the value of the given operand, None otherwise.
	AND -- create a fluent which value is True if both, the value of its parent and the other operand express True, None otherwise.
	OR -- create a fluent which value is True if either the value of its parent or the other operand express True, None otherwise.
	NOT -- create a fluent which value is True if the value of its parent expresses False, None otherwise.
	"""

	def __init__(self, value = None, name = None):
		"""Create a new fluent.

		Arguments:
		value -- the value of the fluent which can also be a function to create user defined operators (default is None).
		name -- the name of the fluent (default is a random string).
		"""
		self._cv = Condition()
		self._mutex = Lock()
		self._pulses = 0
		self._children = []
		self._handle_missed = Behavior.NEVER
		self._value = value

		if name is not None:
			self.name = name
		else:
			self.name = str(uuid4())

	def pulsed(self, handle_missed = 2):
		"""Create a fluent which changes its value from None to True whenever the parent gets pulsed.

		Arguments:
		handle_missed -- see the docstring of the Behavior enumeration to find out more (default is Behavior.ONCE).
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

	def pulse(self):
		"""Pulse a fluent without changing its value."""
		for child in self._children:
			with child._mutex:
				child._pulses += 1

			child.pulse()

		with self._cv:
			self._cv.notify()

	def add_child(self, child):
		"""Add a child to the fluent which gets pulsed whenever this fluent gets pulsed, too.

		Arguments:
		child -- the child to add.
		"""
		self._children.append(child)

	def get_value(self):
		"""Return the value of the fluent."""
		with self._mutex:
			if callable(self._value):
				return self._value()

			return self._value

	def set_value(self, value):
		"""Change the value of the fluent.

		Changing the value will also pulse the fluent.

		Arguments:
		value -- the new value of the fluent.
		"""
		with self._mutex:
			self._value = value

		self.pulse()

	def wait_for(self, timeout = None):
		"""Block the current thread if the value of the fluent is None, until it is not None or until it timed out.

		If the fluent was created by the pulsed method of a fluent, the method blocks until the parent gets pulsed.

		The return value is the last return value of the predicate (value is not None) and will evaluate to False if the method timed out.

		Arguments:
		timeout -- the maximum time to wait (default is None).
		"""
		with self._cv:
			return self._cv.wait_for(lambda: self.get_value() is not None, timeout)

	def _compare(self, operator, other):
		"""This is a helper method for internal usage only.

		Create a fluent which value is a function returning True or None depending on the given comparison operator applied to the operands.

		Arguments:
		operator -- the comparison operator to apply.
		other -- the other operand.
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

	def __lt__(self, other):
		"""Overload the < comparsion operator.

		Arguments:
		other -- the other operand.
		"""
		return self._compare(operator.lt, other)

	def __leq__(self, other):
		"""Overload the <= comparsion operator.

		Arguments:
		other -- the other operand.
		"""
		return self._compare(operator.leq, other)

	def __eq__(self, other):
		"""Overload the == comparsion operator.

		Arguments:
		other -- the other operand.
		"""
		return self._compare(operator.eq, other)

	def __ne__(self, other):
		"""Overload the != comparsion operator.

		Arguments:
		other -- the other operand.
		"""
		return self._compare(operator.ne, other)

	def IS(self, other):
		"""Create a fluent which value is True if the value of its parent is the value of the given operand, None otherwise.

		Arguments:
		other -- the other operand which can also be a fluent.
		"""
		return self._compare(operator.is_, other)

	def IS_NOT(self, other):
		"""Create a fluent which value is True if the value of its parent is not the value of the given operand, None otherwise.

		Arguments:
		other -- the other operand which can also be a fluent.
		"""
		return self._compare(operator.is_not, other)

	def __gt__(self, other):
		"""Overload the > comparsion operator.

		Arguments:
		other -- the other operand.
		"""
		return self._compare(operator.gt, other)

	def __geq__(self, other):
		"""Overload the >= comparsion operator.

		Arguments:
		other -- the other operand.
		"""
		return self._compare(operator.geq, other)

	def _math(self, operator, operand, other):
		"""This is a helper method for internal usage only.

		Create a fluent which value is a function returning the value of the given math operator applied to the operands.

		Arguments:
		operator -- the math operator to apply.
		operand -- the first operand.
		other -- the other operand.
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

	def __add__(self, other):
		"""Overload the + math operator.

		Arguments:
		other -- the other operand.
		"""
		return self._math(operator.add, self, other)

	def __radd__(self, other):
		"""Overload the + math operator with the first operand not being a fluent.

		Arguments:
		other -- the other operand.
		"""
		return self._math(operator.add, other, self)

	def __sub__(self, other):
		"""Overload the - math operator.

		Arguments:
		other -- the other operand.
		"""
		return self._math(operator.sub, self, other)

	def __rsub__(self, other):
		"""Overload the - math operator with the first operand not being a fluent.

		Arguments:
		other -- the other operand.
		"""
		return self._math(operator.sub, other, self)

	def __mul__(self, other):
		"""Overload the * math operator.

		Arguments:
		other -- the other operand.
		"""
		return self._math(operator.mul, self, other)

	def __rmul__(self, other):
		"""Overload the * math operator with the first operand not being a fluent.

		Arguments:
		other -- the other operand.
		"""
		return self._math(operator.mul, other, self)

	def __truediv__(self, other):
		"""Overload the / math operator.

		Arguments:
		other -- the other operand.
		"""
		return self._math(operator.truediv, self, other)

	def __rtruediv__(self, other):
		"""Overload the / math operator with the first operand not being a fluent.

		Arguments:
		other -- the other operand.
		"""
		return self._math(operator.truediv, other, self)

	def AND(self, other):
		"""Create a fluent which value is True if both, the value of its parent and the other operand express True, None otherwise.

		Arguments:
		other -- the other operand which can also be a fluent.
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

	def OR(self, other):
		"""Create a fluent which value is True if either the value of its parent or the other operand express True, None otherwise.

		Arguments:
		other -- the other operand which can also be a fluent.
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

	def NOT(self):
		"""Create a fluent which value is True if the value of its parent expresses False, None otherwise."""
		def value():
			if not self.get_value():
				return True
			else:
				return None

		fluent = Fluent(value)
		self.add_child(fluent)
		return fluent
