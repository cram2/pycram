:py:mod:`pycram.process_modules.hsrb_process_modules`
=====================================================

.. py:module:: pycram.process_modules.hsrb_process_modules


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.process_modules.hsrb_process_modules.HSRBNavigation
   pycram.process_modules.hsrb_process_modules.HSRBMoveHead
   pycram.process_modules.hsrb_process_modules.HSRBMoveGripper
   pycram.process_modules.hsrb_process_modules.HSRBDetecting
   pycram.process_modules.hsrb_process_modules.HSRBMoveTCP
   pycram.process_modules.hsrb_process_modules.HSRBMoveArmJoints
   pycram.process_modules.hsrb_process_modules.HSRBMoveJoints
   pycram.process_modules.hsrb_process_modules.HSRBWorldStateDetecting
   pycram.process_modules.hsrb_process_modules.HSRBOpen
   pycram.process_modules.hsrb_process_modules.HSRBClose
   pycram.process_modules.hsrb_process_modules.HSRBNavigationReal
   pycram.process_modules.hsrb_process_modules.HSRBNavigationSemiReal
   pycram.process_modules.hsrb_process_modules.HSRBMoveHeadReal
   pycram.process_modules.hsrb_process_modules.HSRBDetectingReal
   pycram.process_modules.hsrb_process_modules.HSRBMoveTCPReal
   pycram.process_modules.hsrb_process_modules.HSRBMoveArmJointsReal
   pycram.process_modules.hsrb_process_modules.HSRBMoveJointsReal
   pycram.process_modules.hsrb_process_modules.HSRBMoveGripperReal
   pycram.process_modules.hsrb_process_modules.HSRBOpenReal
   pycram.process_modules.hsrb_process_modules.HSRBCloseReal
   pycram.process_modules.hsrb_process_modules.HSRBManager



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.process_modules.hsrb_process_modules.calculate_and_apply_ik
   pycram.process_modules.hsrb_process_modules._park_arms
   pycram.process_modules.hsrb_process_modules._move_arm_tcp



.. py:function:: calculate_and_apply_ik(robot, gripper: str, target_position: pycram.datastructures.pose.Point, max_iterations: Optional[int] = None)

   Calculates the inverse kinematics for the given target pose and applies it to the robot.


.. py:function:: _park_arms(arm)

   Defines the joint poses for the parking positions of the arms of HSRB and applies them to the
   in the World defined robot.
   :return: None


.. py:class:: HSRBNavigation(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   The process module to move the robot from one position to another.

   Create a new process module.

   .. py:method:: _execute(desig: MoveMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveHead(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module moves the head to look at a specific point in the world coordinate frame.
   This point can either be a position or an object.

   Create a new process module.

   .. py:method:: _execute(desig: LookingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveGripper(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module controls the gripper of the robot. They can either be opened or closed.
   Furthermore, it can only moved one gripper at a time.

   Create a new process module.

   .. py:method:: _execute(desig: MoveGripperMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBDetecting(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module tries to detect an object with the given type. To be detected the object has to be in
   the field of view of the robot.

   Create a new process module.

   .. py:method:: _execute(desig: DetectingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveTCP(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process moves the tool center point of either the right or the left arm.

   Create a new process module.

   .. py:method:: _execute(desig: MoveTCPMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveArmJoints(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process modules moves the joints of either the right or the left arm. The joint states can be given as
   list that should be applied or a pre-defined position can be used, such as "parking"

   Create a new process module.

   .. py:method:: _execute(desig: MoveArmJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveJoints(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot

   Create a new process module.

   .. py:method:: _execute(desig: MoveJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBWorldStateDetecting(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module detectes an object even if it is not in the field of view of the robot.

   Create a new process module.

   .. py:method:: _execute(desig: WorldStateDetectingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBOpen(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.

   Create a new process module.

   .. py:method:: _execute(desig: OpeningMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBClose(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Low-level implementation that lets the robot close a grasped container, in simulation

   Create a new process module.

   .. py:method:: _execute(desig: ClosingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:function:: _move_arm_tcp(target: Pose, robot: pycram.world_concepts.world_object.Object, arm: Arms) -> None


.. py:class:: HSRBNavigationReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base

   Create a new process module.

   .. py:method:: _execute(designator: MoveMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBNavigationSemiReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base

   Create a new process module.

   .. py:method:: _execute(designator: MoveMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveHeadReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
   as the simulated one

   Create a new process module.

   .. py:method:: _execute(desig: LookingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBDetectingReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Process Module for the real HSRB that tries to detect an object fitting the given object description. Uses Robokudo
   for perception of the environment.

   Create a new process module.

   .. py:method:: _execute(desig: DetectingMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveTCPReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Moves the tool center point of the real HSRB while avoiding all collisions

   Create a new process module.

   .. py:method:: _execute(designator: MoveTCPMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveArmJointsReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Moves the arm joints of the real HSRB to the given configuration while avoiding all collisions

   Create a new process module.

   .. py:method:: _execute(designator: MoveArmJointsMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveJointsReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Moves any joint using giskard, avoids all collisions while doint this.

   Create a new process module.

   .. py:method:: _execute(designator: MoveJointsMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveGripperReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Opens or closes the gripper of the real HSRB with the help of giskard.

   Create a new process module.

   .. py:method:: _execute(designator: MoveGripperMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBOpenReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Tries to open an already grasped container

   Create a new process module.

   .. py:method:: _execute(designator: OpeningMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBCloseReal(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Tries to close an already grasped container

   Create a new process module.

   .. py:method:: _execute(designator: ClosingMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBManager


   Bases: :py:obj:`ProcessModuleManager`

   .. py:method:: navigate()


   .. py:method:: looking()


   .. py:method:: detecting()


   .. py:method:: move_tcp()


   .. py:method:: move_arm_joints()


   .. py:method:: world_state_detecting()


   .. py:method:: move_joints()


   .. py:method:: move_gripper()


   .. py:method:: open()


   .. py:method:: close()



