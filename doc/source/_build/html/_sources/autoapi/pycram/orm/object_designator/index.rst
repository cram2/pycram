:py:mod:`pycram.orm.object_designator`
======================================

.. py:module:: pycram.orm.object_designator


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.orm.object_designator.ObjectMixin
   pycram.orm.object_designator.Object
   pycram.orm.object_designator.ObjectPart
   pycram.orm.object_designator.BelieveObject




.. py:class:: ObjectMixin


   Bases: :py:obj:`sqlalchemy.orm.MappedAsDataclass`

   ObjectMixin holds a foreign key column and its relationship to the referenced table.
   For information about Mixins, see https://docs.sqlalchemy.org/en/13/orm/extensions/declarative/mixins.html

   .. py:attribute:: __abstract__
      :value: True

      

   .. py:attribute:: object_to_init
      :type: bool

      

   .. py:method:: object_id() -> sqlalchemy.orm.Mapped[int]


   .. py:method:: object()



.. py:class:: Object


   Bases: :py:obj:`pycram.orm.base.PoseMixin`, :py:obj:`pycram.orm.base.Base`

   ORM class of pycram.designators.object_designator.ObjectDesignator

   .. py:attribute:: dtype
      :type: sqlalchemy.orm.Mapped[str]

      

   .. py:attribute:: obj_type
      :type: sqlalchemy.orm.Mapped[Optional[pycram.datastructures.enums.ObjectType]]

      

   .. py:attribute:: name
      :type: sqlalchemy.orm.Mapped[str]

      

   .. py:attribute:: __mapper_args__

      


.. py:class:: ObjectPart


   Bases: :py:obj:`Object`

   ORM Class of pycram.designators.object_designator.LocatedObject.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: __mapper_args__

      


.. py:class:: BelieveObject


   Bases: :py:obj:`pycram.orm.base.MapperArgsMixin`, :py:obj:`Object`

   ORM class of pycram.designators.object_designator.ObjectDesignator

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      


