:py:mod:`pycram.designators.motion_designator`
==============================================

.. py:module:: pycram.designators.motion_designator


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.designators.motion_designator.BaseMotion
   pycram.designators.motion_designator.MoveMotion
   pycram.designators.motion_designator.MoveTCPMotion
   pycram.designators.motion_designator.LookingMotion
   pycram.designators.motion_designator.MoveGripperMotion
   pycram.designators.motion_designator.DetectingMotion
   pycram.designators.motion_designator.MoveArmJointsMotion
   pycram.designators.motion_designator.WorldStateDetectingMotion
   pycram.designators.motion_designator.MoveJointsMotion
   pycram.designators.motion_designator.OpeningMotion
   pycram.designators.motion_designator.ClosingMotion




.. py:class:: BaseMotion


   Bases: :py:obj:`abc.ABC`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:method:: perform()
      :abstractmethod:

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.Motion
      :abstractmethod:

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session: sqlalchemy.orm.Session, *args, **kwargs) -> pycram.orm.motion_designator.Motion
      :abstractmethod:

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.


   .. py:method:: __post_init__()

      Checks if types are missing or wrong



.. py:class:: MoveMotion


   Bases: :py:obj:`BaseMotion`

   Moves the robot to a designated location

   .. py:attribute:: target
      :type: pycram.datastructures.pose.Pose

      Location to which the robot should be moved

   .. py:method:: perform()

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.MoveMotion

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session, *args, **kwargs) -> pycram.orm.motion_designator.MoveMotion

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.



.. py:class:: MoveTCPMotion


   Bases: :py:obj:`BaseMotion`

   Moves the Tool center point (TCP) of the robot

   .. py:attribute:: target
      :type: pycram.datastructures.pose.Pose

      Target pose to which the TCP should be moved

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      Arm with the TCP that should be moved to the target

   .. py:attribute:: allow_gripper_collision
      :type: typing_extensions.Optional[bool]

      If the gripper can collide with something

   .. py:method:: perform()

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.MoveTCPMotion

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session: sqlalchemy.orm.Session, *args, **kwargs) -> pycram.orm.motion_designator.MoveTCPMotion

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.



.. py:class:: LookingMotion


   Bases: :py:obj:`BaseMotion`

   Lets the robot look at a point

   .. py:attribute:: target
      :type: pycram.datastructures.pose.Pose

      

   .. py:method:: perform()

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.LookingMotion

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session: sqlalchemy.orm.Session, *args, **kwargs) -> pycram.orm.motion_designator.LookingMotion

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.



.. py:class:: MoveGripperMotion


   Bases: :py:obj:`BaseMotion`

   Opens or closes the gripper

   .. py:attribute:: motion
      :type: pycram.datastructures.enums.GripperState

      Motion that should be performed, either 'open' or 'close'

   .. py:attribute:: gripper
      :type: pycram.datastructures.enums.Arms

      Name of the gripper that should be moved

   .. py:attribute:: allow_gripper_collision
      :type: typing_extensions.Optional[bool]

      If the gripper is allowed to collide with something

   .. py:method:: perform()

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.MoveGripperMotion

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session: sqlalchemy.orm.Session, *args, **kwargs) -> pycram.orm.motion_designator.MoveGripperMotion

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.



.. py:class:: DetectingMotion


   Bases: :py:obj:`BaseMotion`

   Tries to detect an object in the FOV of the robot

   .. py:attribute:: object_type
      :type: pycram.datastructures.enums.ObjectType

      Type of the object that should be detected

   .. py:method:: perform()

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.DetectingMotion

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session: sqlalchemy.orm.Session, *args, **kwargs) -> pycram.orm.motion_designator.DetectingMotion

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.



.. py:class:: MoveArmJointsMotion


   Bases: :py:obj:`BaseMotion`

   Moves the joints of each arm into the given position

   .. py:attribute:: left_arm_poses
      :type: typing_extensions.Optional[typing_extensions.Dict[str, float]]

      Target positions for the left arm joints

   .. py:attribute:: right_arm_poses
      :type: typing_extensions.Optional[typing_extensions.Dict[str, float]]

      Target positions for the right arm joints

   .. py:method:: perform()

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.Motion

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session: sqlalchemy.orm.Session, *args, **kwargs) -> pycram.orm.motion_designator.Motion

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.



.. py:class:: WorldStateDetectingMotion


   Bases: :py:obj:`BaseMotion`

   Detects an object based on the world state.

   .. py:attribute:: object_type
      :type: pycram.datastructures.enums.ObjectType

      Object type that should be detected

   .. py:method:: perform()

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.Motion

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session: sqlalchemy.orm.Session, *args, **kwargs) -> pycram.orm.motion_designator.Motion

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.



.. py:class:: MoveJointsMotion


   Bases: :py:obj:`BaseMotion`

   Moves any joint on the robot

   .. py:attribute:: names
      :type: list

      List of joint names that should be moved

   .. py:attribute:: positions
      :type: list

      Target positions of joints, should correspond to the list of names

   .. py:method:: perform()

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.Motion

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session: sqlalchemy.orm.Session, *args, **kwargs) -> pycram.orm.motion_designator.Motion

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.



.. py:class:: OpeningMotion


   Bases: :py:obj:`BaseMotion`

   Designator for opening container

   .. py:attribute:: object_part
      :type: pycram.designators.object_designator.ObjectPart.Object

      Object designator for the drawer handle

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      Arm that should be used

   .. py:method:: perform()

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.OpeningMotion

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session: sqlalchemy.orm.Session, *args, **kwargs) -> pycram.orm.motion_designator.OpeningMotion

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.



.. py:class:: ClosingMotion


   Bases: :py:obj:`BaseMotion`

   Designator for closing a container

   .. py:attribute:: object_part
      :type: pycram.designators.object_designator.ObjectPart.Object

      Object designator for the drawer handle

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      Arm that should be used

   .. py:method:: perform()

      Passes this designator to the process module for execution. Will be overwritten by each motion.


   .. py:method:: to_sql() -> pycram.orm.motion_designator.ClosingMotion

      Create an ORM object that corresponds to this description. Will be overwritten by each motion.

      :return: The created ORM object.


   .. py:method:: insert(session: sqlalchemy.orm.Session, *args, **kwargs) -> pycram.orm.motion_designator.ClosingMotion

      Add and commit this and all related objects to the session.
      Auto-Incrementing primary keys and foreign keys have to be filled by this method.

      :param session: Session with a database that is used to add and commit the objects
      :param args: Possible extra arguments
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM motion.



