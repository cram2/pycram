:py:mod:`pycram.failure_handling`
=================================

.. py:module:: pycram.failure_handling


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.failure_handling.FailureHandling
   pycram.failure_handling.Retry




.. py:class:: FailureHandling(designator_description: pycram.designator.DesignatorDescription)


   Base class for failure handling mechanisms in automated systems or workflows.

   This class provides a structure for implementing different strategies to handle
   failures that may occur during the execution of a plan or process. It is designed
   to be extended by subclasses that implement specific failure handling behaviors.

   Initializes a new instance of the FailureHandling class.

   :param designator_description: The description or context of the task or process for which the failure handling is being set up.

   .. py:method:: perform()
      :abstractmethod:

      Abstract method to perform the failure handling mechanism.

      This method should be overridden in subclasses to implement the specific
      behavior for handling failures.

      :raises NotImplementedError: If the method is not implemented in a subclass.



.. py:class:: Retry(designator_description: pycram.designator.DesignatorDescription, max_tries: int = 3)


   Bases: :py:obj:`FailureHandling`

   A subclass of FailureHandling that implements a retry mechanism.

   This class represents a specific failure handling strategy where the system
   attempts to retry a failed action a certain number of times before giving up.

   .. attribute:: max_tries

      The maximum number of attempts to retry the action.

      :type: int

   Inherits:
       All attributes and methods from the FailureHandling class.

   Overrides:
       perform(): Implements the retry logic.

   Initializes a new instance of the Retry class.

   :param designator_description: The description or context of the task or process for which the retry mechanism is being set up.
   :param max_tries: The maximum number of attempts to retry. Defaults to 3.

   .. py:method:: perform()

      Implementation of the retry mechanism.

      This method attempts to perform the action specified in the designator_description.
      If the action fails, it is retried up to max_tries times. If all attempts fail,
      the last exception is raised.

      :raises PlanFailure: If all retry attempts fail.



