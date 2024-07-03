:py:mod:`pycram.orm.action_designator`
======================================

.. py:module:: pycram.orm.action_designator


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.orm.action_designator.Action
   pycram.orm.action_designator.ParkArmsAction
   pycram.orm.action_designator.NavigateAction
   pycram.orm.action_designator.MoveTorsoAction
   pycram.orm.action_designator.SetGripperAction
   pycram.orm.action_designator.Release
   pycram.orm.action_designator.GripAction
   pycram.orm.action_designator.PickUpAction
   pycram.orm.action_designator.PlaceAction
   pycram.orm.action_designator.TransportAction
   pycram.orm.action_designator.LookAtAction
   pycram.orm.action_designator.DetectAction
   pycram.orm.action_designator.OpenAction
   pycram.orm.action_designator.CloseAction
   pycram.orm.action_designator.GraspingAction
   pycram.orm.action_designator.FaceAtAction




.. py:class:: Action


   Bases: :py:obj:`pycram.orm.base.MapperArgsMixin`, :py:obj:`pycram.orm.base.Designator`

   ORM class of pycram.designators.action_designator.ActionDesignator.
   The purpose of this class is to correctly map the inheritance from the action designator class into the database.
   Inheritance is implemented as Joined Table Inheritance (see https://docs.sqlalchemy.org/en/20/orm/inheritance.html)

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: dtype
      :type: sqlalchemy.orm.Mapped[str]

      

   .. py:attribute:: robot_state_id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: robot_state
      :type: sqlalchemy.orm.Mapped[pycram.orm.base.RobotState]

      


.. py:class:: ParkArmsAction


   Bases: :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.ParkArmsDesignator.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      


.. py:class:: NavigateAction


   Bases: :py:obj:`pycram.orm.base.PoseMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.NavigateAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      


.. py:class:: MoveTorsoAction


   Bases: :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.MoveTorsoAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: position
      :type: sqlalchemy.orm.Mapped[typing_extensions.Optional[float]]

      


.. py:class:: SetGripperAction


   Bases: :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.SetGripperAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: gripper
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      

   .. py:attribute:: motion
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.GripperState]

      


.. py:class:: Release


   Bases: :py:obj:`pycram.orm.object_designator.ObjectMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.Release.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: gripper
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      


.. py:class:: GripAction


   Bases: :py:obj:`pycram.orm.object_designator.ObjectMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.GripAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: gripper
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      

   .. py:attribute:: effort
      :type: sqlalchemy.orm.Mapped[float]

      


.. py:class:: PickUpAction


   Bases: :py:obj:`pycram.orm.object_designator.ObjectMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.PickUpAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      

   .. py:attribute:: grasp
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Grasp]

      


.. py:class:: PlaceAction


   Bases: :py:obj:`pycram.orm.base.PoseMixin`, :py:obj:`pycram.orm.object_designator.ObjectMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.PlaceAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      


.. py:class:: TransportAction


   Bases: :py:obj:`pycram.orm.base.PoseMixin`, :py:obj:`pycram.orm.object_designator.ObjectMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.TransportAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      


.. py:class:: LookAtAction


   Bases: :py:obj:`pycram.orm.base.PoseMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.LookAtAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      


.. py:class:: DetectAction


   Bases: :py:obj:`pycram.orm.object_designator.ObjectMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.DetectAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      


.. py:class:: OpenAction


   Bases: :py:obj:`pycram.orm.object_designator.ObjectMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.OpenAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      


.. py:class:: CloseAction


   Bases: :py:obj:`pycram.orm.object_designator.ObjectMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.CloseAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      


.. py:class:: GraspingAction


   Bases: :py:obj:`pycram.orm.object_designator.ObjectMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.GraspingAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: arm
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.Arms]

      


.. py:class:: FaceAtAction


   Bases: :py:obj:`pycram.orm.base.PoseMixin`, :py:obj:`Action`

   ORM Class of pycram.designators.action_designator.FaceAtAction.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      


