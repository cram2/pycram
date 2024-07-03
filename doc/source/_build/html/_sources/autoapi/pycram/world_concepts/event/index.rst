:py:mod:`pycram.world_concepts.event`
=====================================

.. py:module:: pycram.world_concepts.event


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.world_concepts.event.Event




.. py:class:: Event


   Base implementation of events in PyCRAM.
   Events allow to attach handler methods to events that fire for specific occurences in the world.

   :ivar handler: List of methods that are called when this event is fired.

   .. py:attribute:: __call__

      Allows to directly call the reference.

   .. py:method:: add(handler: typing_extensions.Callable) -> None

      Adds a new handler to the list of handlers. All handler methods are called when this event is fired.
      Handler have to take the event sender as parameter as well as args* which can contain further parameter.

      :param handler: A method that should be added


   .. py:method:: remove(handler: typing_extensions.Callable) -> None

      Removes a method from the list of handlers, the method will not be called when the event is fired.

      :param handler: The method that should be removed.


   .. py:method:: fire(sender: typing_extensions.Any, earg: typing_extensions.Optional[typing_extensions.Any] = None) -> None

      Fire this event, this causes every method to be called with a sender as well as additional args.

      :param sender: The entity that fired the event.
      :param earg: Additional arguments.


   .. py:method:: __iadd__(other: typing_extensions.Callable) -> Event

      Operator overload that allows to add handlers by the '+=' operator.

      :param other: The handler that should be added.
      :return: This instance


   .. py:method:: __isub__(other: typing_extensions.Callable) -> Event

      Operator overload that allows to remove methods as handlers by using the '-=' operator.

      :param other: The method that should be removed as handler.
      :return: This instance



