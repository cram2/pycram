:py:mod:`pycram.process_module`
===============================

.. py:module:: pycram.process_module

.. autoapi-nested-parse::

   Implementation of process modules.

   Classes:
   ProcessModule -- implementation of process modules.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.process_module.ProcessModule
   pycram.process_module.RealRobot
   pycram.process_module.SimulatedRobot
   pycram.process_module.ProcessModuleManager



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.process_module.with_real_robot
   pycram.process_module.with_simulated_robot



Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.process_module.simulated_robot
   pycram.process_module.real_robot


.. py:class:: ProcessModule(lock)


   Implementation of process modules. Process modules are the part that communicate with the outer world to execute
    designators.

   Create a new process module.

   .. py:attribute:: execution_delay
      :value: True

      Adds a delay of 0.5 seconds after executing a process module, to make the execution in simulation more realistic

   .. py:attribute:: block_list
      :value: []

      List of thread ids for which no Process Modules should be executed. This is used as an interrupt mechanism for
      Designators

   .. py:method:: _execute(designator: pycram.designators.motion_designator.BaseMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.


   .. py:method:: execute(designator: pycram.designators.motion_designator.BaseMotion) -> typing_extensions.Any

      Execute the given designator. If there is already another process module of the same kind the `self._lock` will
      lock this thread until the execution of that process module is finished. This implicitly queues the execution of
      process modules.

      :param designator: The designator to execute.
      :return: Return of the Process Module if there is any



.. py:class:: RealRobot


   Management class for executing designators on the real robot. This is intended to be used in a with environment.
   When importing this class an instance is imported instead.

   Example:

   .. code-block:: python

       with real_robot:
           some designators

   .. py:method:: __enter__()

      Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
      sets it to 'real'


   .. py:method:: __exit__(_type, value, traceback)

      Exit method for the 'with' scope, sets the :py:attr:`~ProcessModuleManager.execution_type` to the previously
      used one.


   .. py:method:: __call__()



.. py:class:: SimulatedRobot


   Management class for executing designators on the simulated robot. This is intended to be used in
   a with environment. When importing this class an instance is imported instead.

   Example:

   .. code-block:: python

       with simulated_robot:
           some designators

   .. py:method:: __enter__()

      Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
      sets it to 'simulated'


   .. py:method:: __exit__(_type, value, traceback)

      Exit method for the 'with' scope, sets the :py:attr:`~ProcessModuleManager.execution_type` to the previously
      used one.


   .. py:method:: __call__()



.. py:function:: with_real_robot(func: typing_extensions.Callable) -> typing_extensions.Callable

   Decorator to execute designators in the decorated class on the real robot.

   Example:

   .. code-block:: python

       @with_real_robot
       def plan():
           some designators

   :param func: Function this decorator is annotating
   :return: The decorated function wrapped into the decorator


.. py:function:: with_simulated_robot(func: typing_extensions.Callable) -> typing_extensions.Callable

   Decorator to execute designators in the decorated class on the simulated robot.

   Example:

   .. code-block:: python

       @with_simulated_robot
       def plan():
           some designators

   :param func: Function this decorator is annotating
   :return: The decorated function wrapped into the decorator


.. py:data:: simulated_robot

   

.. py:data:: real_robot

   

.. py:class:: ProcessModuleManager(robot_name)


   Bases: :py:obj:`abc.ABC`

   Base class for managing process modules, any new process modules have to implement this class to register the
   Process Modules

   Registers the Process modules for this robot. The name of the robot has to match the name given in the robot
   description.

   :param robot_name: Name of the robot for which these Process Modules are intended

   .. py:attribute:: execution_type

      Whether the robot for which the process module is intended for is real or a simulated one

   .. py:attribute:: available_pms
      :value: []

      List of all available Process Module Managers

   .. py:attribute:: _instance

      Singelton instance of this Process Module Manager

   .. py:method:: get_manager() -> typing_extensions.Union[ProcessModuleManager, None]
      :staticmethod:

      Returns the Process Module manager for the currently loaded robot or None if there is no Manager.

      :return: ProcessModuleManager instance of the current robot


   .. py:method:: navigate() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for navigating the robot with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for navigating


   .. py:method:: pick_up() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for picking up with respect to the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for picking up an object


   .. py:method:: place() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for placing with respect to the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for placing an Object


   .. py:method:: looking() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for looking at a point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for looking at a specific point


   .. py:method:: detecting() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for detecting an object with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for detecting an object


   .. py:method:: move_tcp() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for moving the Tool Center Point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the TCP


   .. py:method:: move_arm_joints() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for moving the joints of the robot arm
      with respect to the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the arm joints


   .. py:method:: world_state_detecting() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for detecting an object using the world state with respect to the
      :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for world state detecting


   .. py:method:: move_joints() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for moving any joint of the robot with respect to the
      :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving joints


   .. py:method:: move_gripper() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for moving the gripper with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the gripper


   .. py:method:: open() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for opening drawers with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for opening drawers


   .. py:method:: close() -> typing_extensions.Type[ProcessModule]
      :abstractmethod:

      Returns the Process Module for closing drawers with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for closing drawers



