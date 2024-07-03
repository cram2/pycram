:py:mod:`pycram.orm.base`
=========================

.. py:module:: pycram.orm.base

.. autoapi-nested-parse::

   Implementation of base classes for orm modelling.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.orm.base._Base
   pycram.orm.base.Base
   pycram.orm.base.MapperArgsMixin
   pycram.orm.base.PositionMixin
   pycram.orm.base.QuaternionMixin
   pycram.orm.base.PoseMixin
   pycram.orm.base.ProcessMetaData
   pycram.orm.base.Designator
   pycram.orm.base.Position
   pycram.orm.base.Quaternion
   pycram.orm.base.Pose
   pycram.orm.base.Color
   pycram.orm.base.RobotState



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.orm.base.get_pycram_version_from_git



.. py:function:: get_pycram_version_from_git() -> Optional[str]

   Get the PyCRAM commit hash that is used to run this version.

   This assumes that you have gitpython installed and that the PyCRAM git repository on your system can be found
   with "roscd pycram".


.. py:class:: _Base


   Bases: :py:obj:`sqlalchemy.orm.DeclarativeBase`, :py:obj:`sqlalchemy.orm.MappedAsDataclass`

   Dummy class

   .. py:attribute:: type_annotation_map

      

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      Unique integer ID as auto incremented primary key.

   .. py:method:: __tablename__()



.. py:class:: Base


   Bases: :py:obj:`_Base`

   Base class to add orm functionality to all pycram mappings

   .. py:attribute:: __abstract__
      :value: True

      

   .. py:method:: process_metadata_id() -> sqlalchemy.orm.Mapped[int]


   .. py:method:: process_metadata()



.. py:class:: MapperArgsMixin


   Bases: :py:obj:`sqlalchemy.orm.MappedAsDataclass`

   MapperArgsMixin stores __mapper_args__ information for certain subclass-tables.
   For information about Mixins, see https://docs.sqlalchemy.org/en/20/orm/declarative_mixins.html

   .. py:attribute:: __abstract__
      :value: True

      

   .. py:method:: __mapper_args__()



.. py:class:: PositionMixin


   Bases: :py:obj:`sqlalchemy.orm.MappedAsDataclass`

   PositionMixin holds a foreign key column and its relationship to the referenced table.
   For information about Mixins, see https://docs.sqlalchemy.org/en/20/orm/declarative_mixins.html

   .. py:attribute:: __abstract__
      :value: True

      

   .. py:attribute:: position_to_init
      :type: bool

      

   .. py:method:: position_id() -> sqlalchemy.orm.Mapped[int]


   .. py:method:: position()



.. py:class:: QuaternionMixin


   Bases: :py:obj:`sqlalchemy.orm.MappedAsDataclass`

   QuaternionMixin holds a foreign key column and its relationship to the referenced table.
   For information about Mixins, see https://docs.sqlalchemy.org/en/20/orm/declarative_mixins.html

   .. py:attribute:: __abstract__
      :value: True

      

   .. py:attribute:: orientation_to_init
      :type: bool

      

   .. py:method:: orientation_id() -> sqlalchemy.orm.Mapped[int]


   .. py:method:: orientation()



.. py:class:: PoseMixin


   Bases: :py:obj:`sqlalchemy.orm.MappedAsDataclass`

   PoseMixin holds a foreign key column and its relationship to the referenced table.
   For information about Mixins, see https://docs.sqlalchemy.org/en/20/orm/declarative_mixins.html

   .. py:attribute:: __abstract__
      :value: True

      

   .. py:attribute:: pose_to_init
      :type: bool

      

   .. py:method:: pose_id() -> sqlalchemy.orm.Mapped[int]


   .. py:method:: pose()



.. py:class:: ProcessMetaData


   Bases: :py:obj:`_Base`

   ProcessMetaData stores information about the context of this experiment.

   This class is a singleton and only one MetaData can exist per session.

   .. py:attribute:: created_at
      :type: sqlalchemy.orm.Mapped[datetime.datetime]

      The timestamp where this row got created. This is an aid for versioning.

   .. py:attribute:: created_by
      :type: sqlalchemy.orm.Mapped[str]

      The user that created the experiment.

   .. py:attribute:: description
      :type: sqlalchemy.orm.Mapped[str]

      A description of the purpose (?) of this experiment.

   .. py:attribute:: pycram_version
      :type: sqlalchemy.orm.Mapped[str]

      The PyCRAM version used to generate this row.

   .. py:attribute:: _self

      The singleton instance.

   .. py:method:: committed()

      Return if this object is in the database or not.


   .. py:method:: insert(session: sqlalchemy.orm.Session)

      Insert this into the database using the session. Skipped if it already is inserted.


   .. py:method:: reset()
      :classmethod:

      Reset the singleton instance to None, s. t. next time the class is called a new instance is created.



.. py:class:: Designator


   Bases: :py:obj:`Base`

   ORM Class holding every performed action and motion serving as every performables and motions root.

   .. py:method:: dtype() -> sqlalchemy.orm.Mapped[str]


   .. py:method:: __mapper_args__()



.. py:class:: Position


   Bases: :py:obj:`Base`

   ORM Class for 3D positions.

   .. py:attribute:: x
      :type: sqlalchemy.orm.Mapped[float]

      

   .. py:attribute:: y
      :type: sqlalchemy.orm.Mapped[float]

      

   .. py:attribute:: z
      :type: sqlalchemy.orm.Mapped[float]

      


.. py:class:: Quaternion


   Bases: :py:obj:`Base`

   ORM Class for Quaternions.

   .. py:attribute:: x
      :type: sqlalchemy.orm.Mapped[float]

      

   .. py:attribute:: y
      :type: sqlalchemy.orm.Mapped[float]

      

   .. py:attribute:: z
      :type: sqlalchemy.orm.Mapped[float]

      

   .. py:attribute:: w
      :type: sqlalchemy.orm.Mapped[float]

      


.. py:class:: Pose


   Bases: :py:obj:`PositionMixin`, :py:obj:`QuaternionMixin`, :py:obj:`Base`

   ORM Class for Poses.

   .. py:attribute:: time
      :type: sqlalchemy.orm.Mapped[datetime.datetime]

      

   .. py:attribute:: frame
      :type: sqlalchemy.orm.Mapped[str]

      


.. py:class:: Color


   Bases: :py:obj:`Base`

   ORM Class for Colors.

   .. py:attribute:: r
      :type: sqlalchemy.orm.Mapped[float]

      

   .. py:attribute:: g
      :type: sqlalchemy.orm.Mapped[float]

      

   .. py:attribute:: b
      :type: sqlalchemy.orm.Mapped[float]

      

   .. py:attribute:: alpha
      :type: sqlalchemy.orm.Mapped[float]

      


.. py:class:: RobotState


   Bases: :py:obj:`PoseMixin`, :py:obj:`Base`

   ORM Representation of a robots state.

   .. py:attribute:: torso_height
      :type: sqlalchemy.orm.Mapped[float]

      The torso height of the robot.

   .. py:attribute:: type
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.ObjectType]

      The type of the robot.


