:py:mod:`pycram.process_modules.boxy_process_modules`
=====================================================

.. py:module:: pycram.process_modules.boxy_process_modules


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.process_modules.boxy_process_modules.BoxyNavigation
   pycram.process_modules.boxy_process_modules.BoxyOpen
   pycram.process_modules.boxy_process_modules.BoxyClose
   pycram.process_modules.boxy_process_modules.BoxyParkArms
   pycram.process_modules.boxy_process_modules.BoxyMoveHead
   pycram.process_modules.boxy_process_modules.BoxyMoveGripper
   pycram.process_modules.boxy_process_modules.BoxyDetecting
   pycram.process_modules.boxy_process_modules.BoxyMoveTCP
   pycram.process_modules.boxy_process_modules.BoxyMoveArmJoints
   pycram.process_modules.boxy_process_modules.BoxyWorldStateDetecting
   pycram.process_modules.boxy_process_modules.BoxyManager



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.process_modules.boxy_process_modules._park_arms
   pycram.process_modules.boxy_process_modules._move_arm_tcp



.. py:function:: _park_arms(arm)

   Defines the joint poses for the parking positions of the arms of Boxy and applies them to the
   in the BulletWorld defined robot.
   :return: None


.. py:class:: BoxyNavigation(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   The process module to move the robot from one position to another.

   Create a new process module.

   .. py:method:: _execute(desig: MoveMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyOpen(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.

   Create a new process module.

   .. py:method:: _execute(desig: OpeningMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyClose(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Low-level implementation that lets the robot close a grasped container, in simulation

   Create a new process module.

   .. py:method:: _execute(desig: ClosingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyParkArms(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module is for moving the arms in a parking position.
   It is currently not used.

   Create a new process module.

   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyMoveHead(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module moves the head to look at a specific point in the world coordinate frame.
   This point can either be a position or an object.

   Create a new process module.

   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyMoveGripper(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module controls the gripper of the robot. They can either be opened or closed.
   Furthermore, it can only move one gripper at a time.

   Create a new process module.

   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyDetecting(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module tries to detect an object with the given type. To be detected the object has to be in
   the field of view of the robot.

   Create a new process module.

   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyMoveTCP(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process moves the tool center point of either the right or the left arm.

   Create a new process module.

   .. py:method:: _execute(desig: MoveTCPMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyMoveArmJoints(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process modules moves the joints of either the right or the left arm. The joint states can be given as
   list that should be applied or a pre-defined position can be used, such as "parking"

   Create a new process module.

   .. py:method:: _execute(desig: MoveArmJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyWorldStateDetecting(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module detectes an object even if it is not in the field of view of the robot.

   Create a new process module.

   .. py:method:: _execute(desig: WorldStateDetectingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:function:: _move_arm_tcp(target: Pose, robot: pycram.world_concepts.world_object.Object, arm: Arms) -> None


.. py:class:: BoxyManager


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



