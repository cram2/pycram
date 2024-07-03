:py:mod:`pycram.fluent`
=======================

.. py:module:: pycram.fluent


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.fluent.Behavior
   pycram.fluent.Fluent




.. py:class:: Behavior


   Bases: :py:obj:`enum.Enum`

   Enumeration which can be passed as argument to the pulsed method of fluents to describe how to handle missed pulses in the whenever macro.

   Fields:
   NEVER -- ignore missed pulses.
   ONCE -- if pulses were missed, execute the body once more in total.
   ALWAYS -- if pulses were missed, execute the body once more for each.

   .. py:attribute:: NEVER
      :value: 1

      

   .. py:attribute:: ONCE
      :value: 2

      

   .. py:attribute:: ALWAYS
      :value: 3

      


.. py:class:: Fluent(value: typing_extensions.Optional[typing_extensions.Any] = None, name: str = None)


   Implementation of fluents.

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

   Create a new fluent.

   :param value:  the value of the fluent which can also be a function to create user defined operators (default is None).
   :param name: the name of the fluent (default is a random string).

   .. py:method:: pulsed(handle_missed: Behavior = 2) -> Fluent

      Create a fluent which changes its value from None to True whenever the parent gets pulsed.

      :param handle_missed: see the docstring of the Behavior enumeration to find out more (default is Behavior.ONCE).


   .. py:method:: pulse() -> None

      Pulse a fluent without changing its value.


   .. py:method:: whenever(callback: typing_extensions.Callable) -> None

      Registers a callback which is called everytime this Fluent is pulsed. The callback should be a Callable. When
      the callback is called it gets the current value of this Fluent as an argument.

      :param callback: The callback which should be called when pulsed as a Callable.


   .. py:method:: add_child(child: Fluent) -> None

      Add a child to the fluent which gets pulsed whenever this fluent gets pulsed, too.

      :param child: the child to add.


   .. py:method:: get_value() -> typing_extensions.Any

      Return the value of the fluent.


   .. py:method:: set_value(value: typing_extensions.Any) -> None

      Change the value of the fluent.
      Changing the value will also pulse the fluent.

      :param value: the new value of the fluent.


   .. py:method:: wait_for(timeout: typing_extensions.Optional[float] = None)

      Block the current thread if the value of the fluent is None, until it is not None or until it timed out.

      If the fluent was created by the pulsed method of a fluent, the method blocks until the parent gets pulsed.

      The return value is the last return value of the predicate (value is not None) and will evaluate to False if the method timed out.

      :param timeout: the maximum time to wait (default is None).


   .. py:method:: _compare(operator, other: Fluent) -> Fluent

      This is a helper method for internal usage only.

      Create a fluent which value is a function returning True or None depending on the given comparison operator applied to the operands.

      :param operator: the comparison operator to apply.
      :param other: the other operand.


   .. py:method:: __lt__(other: Fluent) -> Fluent

      Overload the < comparsion operator.

      :param other: -- the other operand.


   .. py:method:: __leq__(other: Fluent) -> Fluent

      Overload the <= comparsion operator.

      :param other: the other operand.


   .. py:method:: __eq__(other: Fluent) -> Fluent

      Overload the == comparsion operator.

      :param other: the other operand.


   .. py:method:: __ne__(other: Fluent) -> Fluent

      Overload the != comparsion operator.

      :param other: the other operand.


   .. py:method:: IS(other: Fluent) -> Fluent

      Create a fluent which value is True if the value of its parent is the value of the given operand, None otherwise.

      :param other: -- the other operand which can also be a fluent.


   .. py:method:: IS_NOT(other: Fluent) -> Fluent

      Create a fluent which value is True if the value of its parent is not the value of the given operand, None otherwise.

      :param other: -- the other operand which can also be a fluent.


   .. py:method:: __gt__(other: Fluent) -> Fluent

      Overload the > comparison operator.

      :param other: the other operand.


   .. py:method:: __geq__(other: Fluent) -> Fluent

      Overload the >= comparison operator.

      :param other: the other operand.


   .. py:method:: _math(operator: typing_extensions.Callable, operand: Fluent, other: Fluent) -> Fluent

      This is a helper method for internal usage only.

      Create a fluent which value is a function returning the value of the given math operator applied to the operands.

      :param operator: the math operator to apply.
      :param operand: the first operand.
      :param other: the other operand.


   .. py:method:: __add__(other: Fluent) -> Fluent

      Overload the + math operator.

      :parm other: the other operand.


   .. py:method:: __radd__(other: Fluent) -> Fluent

      Overload the + math operator with the first operand not being a fluent.

      :param other: the other operand.


   .. py:method:: __sub__(other: Fluent) -> Fluent

      Overload the - math operator.

      :param other: the other operand.


   .. py:method:: __rsub__(other: Fluent) -> Fluent

      Overload the - math operator with the first operand not being a fluent.

      :param other: the other operand.


   .. py:method:: __mul__(other: Fluent) -> Fluent

      Overload the * math operator.

      :param other: the other operand.


   .. py:method:: __rmul__(other: Fluent) -> Fluent

      Overload the * math operator with the first operand not being a fluent.

      :param other: the other operand.


   .. py:method:: __truediv__(other: Fluent) -> Fluent

      Overload the / math operator.

      :param other: the other operand.


   .. py:method:: __rtruediv__(other) -> Fluent

      Overload the / math operator with the first operand not being a fluent.

      :param other: the other operand.


   .. py:method:: AND(other: Fluent) -> Fluent

      Create a fluent which value is True if both, the value of its parent and the other operand express True, None otherwise.

      :param other: the other operand which can also be a fluent.


   .. py:method:: OR(other: Fluent) -> Fluent

      Create a fluent which value is True if either the value of its parent or the other operand express True, None otherwise.

      :param other: the other operand which can also be a fluent.


   .. py:method:: NOT() -> Fluent

      Create a fluent which value is True if the value of its parent expresses False, None otherwise.



