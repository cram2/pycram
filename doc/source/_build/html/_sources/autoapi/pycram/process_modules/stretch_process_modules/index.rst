:py:mod:`pycram.process_modules.stretch_process_modules`
========================================================

.. py:module:: pycram.process_modules.stretch_process_modules


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.process_modules.stretch_process_modules.StretchNavigate
   pycram.process_modules.stretch_process_modules.StretchMoveHead
   pycram.process_modules.stretch_process_modules.StretchMoveGripper
   pycram.process_modules.stretch_process_modules.StretchDetecting
   pycram.process_modules.stretch_process_modules.StretchMoveTCP
   pycram.process_modules.stretch_process_modules.StretchMoveArmJoints
   pycram.process_modules.stretch_process_modules.StretchMoveJoints
   pycram.process_modules.stretch_process_modules.StretchWorldStateDetecting
   pycram.process_modules.stretch_process_modules.StretchOpen
   pycram.process_modules.stretch_process_modules.StretchClose
   pycram.process_modules.stretch_process_modules.StretchNavigationReal
   pycram.process_modules.stretch_process_modules.StretchMoveHeadReal
   pycram.process_modules.stretch_process_modules.StretchDetectingReal
   pycram.process_modules.stretch_process_modules.StretchMoveTCPReal
   pycram.process_modules.stretch_process_modules.StretchMoveArmJointsReal
   pycram.process_modules.stretch_process_modules.StretchMoveJointsReal
   pycram.process_modules.stretch_process_modules.StretchMoveGripperReal
   pycram.process_modules.stretch_process_modules.StretchOpenReal
   pycram.process_modules.stretch_process_modules.StretchCloseReal
   pycram.process_modules.stretch_process_modules.StretchManager



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.process_modules.stretch_process_modules._move_arm_tcp



.. py:class:: StretchNavigate


   Bases: :py:obj:`DefaultNavigation`

   Process module for the simulated Stretch that sends a cartesian goal to the robot to move the robot base


.. py:class:: StretchMoveHead


   Bases: :py:obj:`ProcessModule`

   Process module for the simulated Stretch that moves the head such that it looks at the given position

   .. py:method:: _execute(designator: MoveMotion) -> Any



.. py:class:: StretchMoveGripper


   Bases: :py:obj:`DefaultMoveGripper`

   Process module for the simulated Stretch that opens or closes the gripper


.. py:class:: StretchDetecting


   Bases: :py:obj:`DefaultDetecting`

   Process Module for the simulated Stretch that tries to detect an object fitting the given object description


.. py:class:: StretchMoveTCP


   Bases: :py:obj:`DefaultMoveTCP`

   Process module for the simulated Stretch that moves the tool center point of the robot


.. py:class:: StretchMoveArmJoints


   Bases: :py:obj:`DefaultMoveArmJoints`

   Process module for the simulated Stretch that moves the arm joints of the robot


.. py:class:: StretchMoveJoints


   Bases: :py:obj:`DefaultMoveJoints`

   Process module for the simulated Stretch that moves any joint of the robot


.. py:class:: StretchWorldStateDetecting


   Bases: :py:obj:`DefaultWorldStateDetecting`

   Process Module for the simulated Stretch that tries to detect an object using the world state


.. py:class:: StretchOpen


   Bases: :py:obj:`ProcessModule`

   Process module for the simulated Stretch that opens an already grasped container

   .. py:method:: _execute(desig: OpeningMotion)



.. py:class:: StretchClose


   Bases: :py:obj:`ProcessModule`

   Process module for the simulated Stretch that closes an already grasped container

   .. py:method:: _execute(desig: ClosingMotion)



.. py:function:: _move_arm_tcp(target: Pose, robot: Object, arm: Arms) -> None


.. py:class:: StretchNavigationReal


   Bases: :py:obj:`ProcessModule`

   Process module for the real Stretch that sends a cartesian goal to giskard to move the robot base

   .. py:method:: _execute(designator: MoveMotion) -> Any



.. py:class:: StretchMoveHeadReal


   Bases: :py:obj:`ProcessModule`

   Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
   as the simulated one

   .. py:method:: _execute(desig: LookingMotion)



.. py:class:: StretchDetectingReal


   Bases: :py:obj:`ProcessModule`

   Process Module for the real Stretch that tries to detect an object fitting the given object description. Uses Robokudo
   for perception of the environment.

   .. py:method:: _execute(designator: DetectingMotion) -> Any



.. py:class:: StretchMoveTCPReal


   Bases: :py:obj:`ProcessModule`

   Moves the tool center point of the real Stretch while avoiding all collisions

   .. py:method:: _execute(designator: MoveTCPMotion) -> Any



.. py:class:: StretchMoveArmJointsReal


   Bases: :py:obj:`ProcessModule`

   Moves the arm joints of the real Stretch to the given configuration while avoiding all collisions

   .. py:method:: _execute(designator: MoveArmJointsMotion) -> Any



.. py:class:: StretchMoveJointsReal


   Bases: :py:obj:`ProcessModule`

   Moves any joint using giskard, avoids all collisions while doint this.

   .. py:method:: _execute(designator: MoveJointsMotion) -> Any



.. py:class:: StretchMoveGripperReal


   Bases: :py:obj:`ProcessModule`

   Opens or closes the gripper of the real Stretch, gripper uses an action server for this instead of giskard

   .. py:method:: _execute(designator: MoveGripperMotion) -> Any



.. py:class:: StretchOpenReal


   Bases: :py:obj:`ProcessModule`

   Tries to open an already grasped container

   .. py:method:: _execute(designator: OpeningMotion) -> Any



.. py:class:: StretchCloseReal


   Bases: :py:obj:`ProcessModule`

   Tries to close an already grasped container

   .. py:method:: _execute(designator: ClosingMotion) -> Any



.. py:class:: StretchManager


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



