:py:mod:`pycram.process_modules`
================================

.. py:module:: pycram.process_modules


Submodules
----------
.. toctree::
   :titlesonly:
   :maxdepth: 1

   boxy_process_modules/index.rst
   default_process_modules/index.rst
   donbot_process_modules/index.rst
   hsrb_process_modules/index.rst
   pr2_process_modules/index.rst
   stretch_process_modules/index.rst


Package Contents
----------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.process_modules.Pr2Manager
   pycram.process_modules.BoxyManager
   pycram.process_modules.DonbotManager
   pycram.process_modules.HSRBManager
   pycram.process_modules.DefaultManager
   pycram.process_modules.StretchManager




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



