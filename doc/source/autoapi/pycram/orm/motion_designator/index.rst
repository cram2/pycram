:py:mod:`pycram.orm.motion_designator`
======================================

.. py:module:: pycram.orm.motion_designator

.. autoapi-nested-parse::

   This module defines a set of ORM classes related to motion designators in the pycram framework.

   Each motion designator class has its own table in the database with columns representing its attributes.
   The MotionDesignator class is the base class that defines the polymorphic behavior of all other motion designator
   classes.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.orm.motion_designator.Motion
   pycram.orm.motion_designator.MoveMotion
   pycram.orm.motion_designator.AccessingMotion
   pycram.orm.motion_designator.MoveTCPMotion
   pycram.orm.motion_designator.LookingMotion
   pycram.orm.motion_designator.MoveGripperMotion
   pycram.orm.motion_designator.DetectingMotion
   pycram.orm.motion_designator.WorldStateDetectingMotion
   pycram.orm.motion_designator.OpeningMotion
   pycram.orm.motion_designator.ClosingMotion




.. py:class:: Motion


   Bases: :py:obj:`pycram.orm.base.MapperArgsMixin`, :py:obj:`pycram.orm.base.Designator`

   ORM class of pycram.designators.motion_designator.MotionDesignatorDescription

   :ivar id: (Integer) Auto-incrementing primary key
   :ivar dtype: (String) Polymorphic discriminator

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: dtype
      :type: sqlalchemy.orm.Mapped[str]

      


.. py:class:: MoveMotion


   Bases: :py:obj:`pycram.orm.base.PoseMixin`, :py:obj:`Motion`

   ORM class of pycram.designators.motion_designator.MoveMotion

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      


.. py:class:: AccessingMotion


   Bases: :py:obj:`Motion`

   ORM class of pycram.designators.motion_designator.AccessingMotion

   :ivar arm: (String) Name of the arm used
   :ivar gripper: (String) Name of the gripper used
   :ivar distance: (Float) Distance from the drawer to the robot
   :ivar drawer_joint:

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: part_of
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: object
      :type: sqlalchemy.orm.Mapped[pycram.orm.object_designator.Object]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      

   .. py:attribute:: gripper
      :type: sqlalchemy.orm.Mapped[str]

      

   .. py:attribute:: distance
      :type: sqlalchemy.orm.Mapped[float]

      

   .. py:attribute:: drawer_joint
      :type: sqlalchemy.orm.Mapped[str]

      

   .. py:attribute:: drawer_handle
      :type: sqlalchemy.orm.Mapped[str]

      


.. py:class:: MoveTCPMotion


   Bases: :py:obj:`pycram.orm.base.PoseMixin`, :py:obj:`Motion`

   ORM class of pycram.designators.motion_designator.MoveTCPMotion

   :ivar arm: String specifying which arm to move the TCP of

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      

   .. py:attribute:: allow_gripper_collision
      :type: sqlalchemy.orm.Mapped[typing_extensions.Optional[bool]]

      


.. py:class:: LookingMotion


   Bases: :py:obj:`pycram.orm.base.PoseMixin`, :py:obj:`Motion`

   ORM class of pycram.designators.motion_designator.LookingMotion

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      


.. py:class:: MoveGripperMotion


   Bases: :py:obj:`Motion`

   ORM class of pycram.designators.motion_designator.MoveGripperMotion

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: motion
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.GripperState]

      

   .. py:attribute:: gripper
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      

   .. py:attribute:: allow_gripper_collision
      :type: sqlalchemy.orm.Mapped[typing_extensions.Optional[bool]]

      


.. py:class:: DetectingMotion


   Bases: :py:obj:`Motion`

   ORM class of pycram.designators.motion_designator.DetectingMotion

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: object_type
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.ObjectType]

      


.. py:class:: WorldStateDetectingMotion


   Bases: :py:obj:`Motion`

   ORM class of pycram.designators.motion_designator.WorldStateDetectingMotion

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: object_type
      :type: sqlalchemy.orm.Mapped[str]

      


.. py:class:: OpeningMotion


   Bases: :py:obj:`Motion`

   ORM class of pycram.designators.motion_designator.OpeningMotion

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      


.. py:class:: ClosingMotion


   Bases: :py:obj:`Motion`

   ORM class of pycram.designators.motion_designator.ClosingMotion

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      


