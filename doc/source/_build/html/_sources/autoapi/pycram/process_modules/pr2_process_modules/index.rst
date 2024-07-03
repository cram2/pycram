:py:mod:`pycram.process_modules.pr2_process_modules`
====================================================

.. py:module:: pycram.process_modules.pr2_process_modules


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.process_modules.pr2_process_modules.Pr2Navigation
   pycram.process_modules.pr2_process_modules.Pr2MoveHead
   pycram.process_modules.pr2_process_modules.Pr2MoveGripper
   pycram.process_modules.pr2_process_modules.Pr2Detecting
   pycram.process_modules.pr2_process_modules.Pr2MoveTCP
   pycram.process_modules.pr2_process_modules.Pr2MoveArmJoints
   pycram.process_modules.pr2_process_modules.PR2MoveJoints
   pycram.process_modules.pr2_process_modules.Pr2WorldStateDetecting
   pycram.process_modules.pr2_process_modules.Pr2Open
   pycram.process_modules.pr2_process_modules.Pr2Close
   pycram.process_modules.pr2_process_modules.Pr2NavigationReal
   pycram.process_modules.pr2_process_modules.Pr2MoveHeadReal
   pycram.process_modules.pr2_process_modules.Pr2DetectingReal
   pycram.process_modules.pr2_process_modules.Pr2MoveTCPReal
   pycram.process_modules.pr2_process_modules.Pr2MoveArmJointsReal
   pycram.process_modules.pr2_process_modules.Pr2MoveJointsReal
   pycram.process_modules.pr2_process_modules.Pr2MoveGripperReal
   pycram.process_modules.pr2_process_modules.Pr2OpenReal
   pycram.process_modules.pr2_process_modules.Pr2CloseReal
   pycram.process_modules.pr2_process_modules.Pr2Manager



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.process_modules.pr2_process_modules._move_arm_tcp



.. py:class:: Pr2Navigation(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   The process module to move the robot from one position to another.

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.MoveMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2MoveHead(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module moves the head to look at a specific point in the world coordinate frame.
   This point can either be a position or an object.

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.LookingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2MoveGripper(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module controls the gripper of the robot. They can either be opened or closed.
   Furthermore, it can only moved one gripper at a time.

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.MoveGripperMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2Detecting(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module tries to detect an object with the given type. To be detected the object has to be in
   the field of view of the robot.

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.DetectingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2MoveTCP(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process moves the tool center point of either the right or the left arm.

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.MoveTCPMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2MoveArmJoints(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process modules moves the joints of either the right or the left arm. The joint states can be given as
   list that should be applied or a pre-defined position can be used, such as "parking"

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.MoveArmJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: PR2MoveJoints(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.MoveJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2WorldStateDetecting(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module detectes an object even if it is not in the field of view of the robot.

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.WorldStateDetectingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2Open(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.OpeningMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2Close(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Low-level implementation that lets the robot close a grasped container, in simulation

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.ClosingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:function:: _move_arm_tcp(target: pycram.datastructures.pose.Pose, robot: pycram.world_concepts.world_object.Object, arm: pycram.datastructures.enums.Arms) -> None


.. py:class:: Pr2NavigationReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Process module for the real PR2 that sends a cartesian goal to giskard to move the robot base

   Create a new process module.

   .. py:method:: _execute(designator: pycram.designators.motion_designator.MoveMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2MoveHeadReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
   as the simulated one

   Create a new process module.

   .. py:method:: _execute(desig: pycram.designators.motion_designator.LookingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2DetectingReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Process Module for the real Pr2 that tries to detect an object fitting the given object description. Uses Robokudo
   for perception of the environment.

   Create a new process module.

   .. py:method:: _execute(designator: pycram.designators.motion_designator.DetectingMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2MoveTCPReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Moves the tool center point of the real PR2 while avoiding all collisions

   Create a new process module.

   .. py:method:: _execute(designator: pycram.designators.motion_designator.MoveTCPMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2MoveArmJointsReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Moves the arm joints of the real PR2 to the given configuration while avoiding all collisions

   Create a new process module.

   .. py:method:: _execute(designator: pycram.designators.motion_designator.MoveArmJointsMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2MoveJointsReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Moves any joint using giskard, avoids all collisions while doint this.

   Create a new process module.

   .. py:method:: _execute(designator: pycram.designators.motion_designator.MoveJointsMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2MoveGripperReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Opens or closes the gripper of the real PR2, gripper uses an action server for this instead of giskard

   Create a new process module.

   .. py:method:: _execute(designator: pycram.designators.motion_designator.MoveGripperMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2OpenReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Tries to open an already grasped container

   Create a new process module.

   .. py:method:: _execute(designator: pycram.designators.motion_designator.OpeningMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2CloseReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Tries to close an already grasped container

   Create a new process module.

   .. py:method:: _execute(designator: pycram.designators.motion_designator.ClosingMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2Manager


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


   .. py:method:: move_joints()

      Returns the Process Module for moving any joint of the robot with respect to the
      :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving joints


   .. py:method:: move_gripper()

      Returns the Process Module for moving the gripper with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the gripper


   .. py:method:: open()

      Returns the Process Module for opening drawers with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for opening drawers


   .. py:method:: close()

      Returns the Process Module for closing drawers with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for closing drawers



