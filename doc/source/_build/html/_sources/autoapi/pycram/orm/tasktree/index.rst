:py:mod:`pycram.orm.tasktree`
=============================

.. py:module:: pycram.orm.tasktree

.. autoapi-nested-parse::

   Implementation of ORM classes associated with pycram.task.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.orm.tasktree.TaskTreeNode




.. py:class:: TaskTreeNode


   Bases: :py:obj:`pycram.orm.base.Base`

   ORM equivalent of pycram.task.TaskTreeNode.

   .. py:attribute:: id
      :type: sqlalchemy.orm.Mapped[int]

      

   .. py:attribute:: action_id
      :type: sqlalchemy.orm.Mapped[typing_extensions.Optional[int]]

      

   .. py:attribute:: action
      :type: sqlalchemy.orm.Mapped[typing_extensions.Optional[pycram.orm.base.Designator]]

      

   .. py:attribute:: start_time
      :type: sqlalchemy.orm.Mapped[datetime.datetime]

      

   .. py:attribute:: end_time
      :type: sqlalchemy.orm.Mapped[typing_extensions.Optional[datetime.datetime]]

      

   .. py:attribute:: status
      :type: sqlalchemy.orm.Mapped[pycram.datastructures.enums.TaskStatus]

      

   .. py:attribute:: reason
      :type: sqlalchemy.orm.Mapped[typing_extensions.Optional[str]]

      

   .. py:attribute:: parent_id
      :type: sqlalchemy.orm.Mapped[typing_extensions.Optional[int]]

      

   .. py:attribute:: parent
      :type: sqlalchemy.orm.Mapped[typing_extensions.Optional[TaskTreeNode]]

      


