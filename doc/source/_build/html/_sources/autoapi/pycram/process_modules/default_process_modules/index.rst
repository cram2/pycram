:py:mod:`pycram.process_modules.default_process_modules`
========================================================

.. py:module:: pycram.process_modules.default_process_modules


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.process_modules.default_process_modules.DefaultNavigation
   pycram.process_modules.default_process_modules.DefaultMoveHead
   pycram.process_modules.default_process_modules.DefaultMoveGripper
   pycram.process_modules.default_process_modules.DefaultDetecting
   pycram.process_modules.default_process_modules.DefaultMoveTCP
   pycram.process_modules.default_process_modules.DefaultMoveArmJoints
   pycram.process_modules.default_process_modules.DefaultMoveJoints
   pycram.process_modules.default_process_modules.DefaultWorldStateDetecting
   pycram.process_modules.default_process_modules.DefaultOpen
   pycram.process_modules.default_process_modules.DefaultClose
   pycram.process_modules.default_process_modules.DefaultManager



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.process_modules.default_process_modules._move_arm_tcp



.. py:class:: DefaultNavigation(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   The process module to move the robot from one position to another.

   Create a new process module.

   .. py:method:: _execute(desig: MoveMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveHead(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module moves the head to look at a specific point in the world coordinate frame.
   This point can either be a position or an object.

   Create a new process module.

   .. py:method:: _execute(desig: LookingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveGripper(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module controls the gripper of the robot. They can either be opened or closed.
   Furthermore, it can only moved one gripper at a time.

   Create a new process module.

   .. py:method:: _execute(desig: MoveGripperMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultDetecting(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process module tries to detect an object with the given type. To be detected the object has to be in
   the field of view of the robot.

   Create a new process module.

   .. py:method:: _execute(desig: DetectingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveTCP(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process moves the tool center point of either the right or the left arm.

   Create a new process module.

   .. py:method:: _execute(desig: MoveTCPMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveArmJoints(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process modules moves the joints of either the right or the left arm. The joint states can be given as
   list that should be applied or a pre-defined position can be used, such as "parking"

   Create a new process module.

   .. py:method:: _execute(desig: MoveArmJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveJoints(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Implementation of process modules. Process modules are the part that communicate with the outer world to execute
    designators.

   Create a new process module.

   .. py:method:: _execute(desig: MoveJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultWorldStateDetecting(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   This process moduledetectes an object even if it is not in the field of view of the robot.

   Create a new process module.

   .. py:method:: _execute(desig: WorldStateDetectingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultOpen(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.

   Create a new process module.

   .. py:method:: _execute(desig: OpeningMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultClose(lock)


   Bases: :py:obj:`pycram.process_module.ProcessModule`

   Low-level implementation that lets the robot close a grasped container, in simulation

   Create a new process module.

   .. py:method:: _execute(desig: ClosingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:function:: _move_arm_tcp(target: Pose, robot: pycram.world_concepts.world_object.Object, arm: Arms) -> None


.. py:class:: DefaultManager


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



