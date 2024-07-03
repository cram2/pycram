:py:mod:`pycram.orm.views`
==========================

.. py:module:: pycram.orm.views


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.orm.views.CreateView
   pycram.orm.views.DropView
   pycram.orm.views.PickUpWithContextView



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.orm.views._create_view
   pycram.orm.views._drop_view
   pycram.orm.views.view_exists
   pycram.orm.views.view_doesnt_exist
   pycram.orm.views.view



Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.orm.views.base


.. py:class:: CreateView(name: str, selectable: sqlalchemy.Select)


   Bases: :py:obj:`sqlalchemy.ExecutableDDLElement`

   Class that is used to create a view. Every instance will be compiled into a SQL CREATE VIEW statement.


.. py:class:: DropView(name: str)


   Bases: :py:obj:`sqlalchemy.ExecutableDDLElement`

   Class that is used to drop a view. Every instance will be compiled into a SQL DROP VIEW statement.


.. py:function:: _create_view(element: CreateView, compiler, **kw) -> str

   Compiles a CreateView instance into a SQL CREATE VIEW statement.
   :param element: CreateView instance
   :param compiler: compiler
   :param kw: keyword arguments
   :return: SQL CREATE VIEW statement


.. py:function:: _drop_view(element: DropView, compiler, **kw) -> str

   Compiles a DropView instance into a SQL DROP VIEW statement.
   :param element: DropView instance
   :param compiler: compiler
   :param kw: keyword arguments
   :return: SQL DROP VIEW statement


.. py:function:: view_exists(ddl: typing_extensions.Union[CreateView, DropView], target, connection: sqlalchemy.engine, **kw) -> bool

   Check if a view exists.
   :param ddl: ddl instance
   :param target: target object
   :param connection: connection
   :param kw: keyword arguments
   :return: True if the view exists, False otherwise


.. py:function:: view_doesnt_exist(ddl: typing_extensions.Union[CreateView, DropView], target, connection: sqlalchemy.engine, **kw) -> bool

   Check if a view does not exist.
   :param ddl: ddl instance
   :param target: target object
   :param connection: connection
   :param kw: keyword arguments
   :return: True if the view does not exist, False otherwise


.. py:function:: view(name: str, metadata: sqlalchemy.MetaData, selectable: sqlalchemy.Select) -> sqlalchemy.TableClause

   Function used to control view creation and deletion. It will listen to the after_create and before_drop events
   of the metadata object in order to either create or drop the view. The view needs to have a column id.


.. py:data:: base

   

.. py:class:: PickUpWithContextView


   Bases: :py:obj:`base`

   View for pickup performables with context.

   .. py:attribute:: __robot_position
      :type: pycram.orm.base.Position

      3D Vector of robot position

   .. py:attribute:: __robot_pose
      :type: pycram.orm.base.Pose

      Complete robot pose

   .. py:attribute:: __object_position
      :type: pycram.orm.base.Position

      3D Vector for object position

   .. py:attribute:: __relative_x

      Distance on x axis between robot and object

   .. py:attribute:: __relative_y

      Distance on y axis between robot and object

   .. py:attribute:: __table__

      


