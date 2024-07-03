:py:mod:`pycram.process_modules.donbot_process_modules`
=======================================================

.. py:module:: pycram.process_modules.donbot_process_modules


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.process_modules.donbot_process_modules.DonbotNavigation
   pycram.process_modules.donbot_process_modules.DonbotPlace
   pycram.process_modules.donbot_process_modules.DonbotMoveHead
   pycram.process_modules.donbot_process_modules.DonbotMoveGripper
   pycram.process_modules.donbot_process_modules.DonbotMoveTCP
   pycram.process_modules.donbot_process_modules.DonbotMoveJoints
   pycram.process_modules.donbot_process_modules.DonbotWorldStateDetecting
   pycram.process_modules.donbot_process_modules.DonbotManager



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.process_modules.donbot_process_modules._park_arms



.. py:function:: _park_arms(arm)

   Defines the joint poses for the parking positions of the arm of Donbot and applies them to the
   in the World defined robot.
   :return: None


.. py:class:: DonbotNavigation(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   The process module to move the robot from one position to another.

   Create a new process module.

   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DonbotPlace(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module places an object at the given position in world coordinate frame.

   Create a new process module.

   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DonbotMoveHead(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module moves the head to look at a specific point in the world coordinate frame.
   This point can either be a position or an object.

   Create a new process module.

   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DonbotMoveGripper(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module controls the gripper of the robot. They can either be opened or closed.
   Furthermore, it can only move one gripper at a time.

   Create a new process module.

   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DonbotMoveTCP(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process moves the tool center point of either the right or the left arm.

   Create a new process module.

   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DonbotMoveJoints(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process modules moves the joints of either the right or the left arm. The joint states can be given as
   list that should be applied or a pre-defined position can be used, such as "parking"

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.MoveArmJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DonbotWorldStateDetecting(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module detectes an object even if it is not in the field of view of the robot.

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.WorldStateDetectingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DonbotManager


   Bases: :py:obj:`pycram.process_module.ProcessModuleManager`

   Base class for managing process modules, any new process modules have to implement this class to register the
   Process Modules

   Registers the Process modules for this robot. The name of the robot has to match the name given in the robot
   description.

   :param robot_name: Name of the robot for which these Process Modules are intended

   .. py:method:: navigate()

      Returns the Process Module for navigating the robot with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for navigating


   .. py:method:: place()

      Returns the Process Module for placing with respect to the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for placing an Object


   .. py:method:: looking()

      Returns the Process Module for looking at a point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for looking at a specific point


   .. py:method:: detecting()

      Returns the Process Module for detecting an object with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for detecting an object


   .. py:method:: move_tcp()

      Returns the Process Module for moving the Tool Center Point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the TCP


   .. py:method:: move_arm_joints()

      Returns the Process Module for moving the joints of the robot arm
      with respect to the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the arm joints


   .. py:method:: world_state_detecting()

      Returns the Process Module for detecting an object using the world state with respect to the
      :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for world state detecting


   .. py:method:: move_gripper()

      Returns the Process Module for moving the gripper with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the gripper



