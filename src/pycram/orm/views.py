# from sqlalchemy.orm import declarative_base, Mapped, column_property
# from typing_extensions import Union
# import sqlalchemy.orm
# from sqlalchemy import table, inspect, event, select, engine, MetaData, Select, TableClause, ExecutableDDLElement
# from sqlalchemy.ext.compiler import compiles
# from .object_designator import Object
#
#
# class CreateView(ExecutableDDLElement):
#     """
#     Class that is used to create a view. Every instance will be compiled into a ORM CREATE VIEW statement.
#     """
#
#     def __init__(self, name: str, selectable: Select):
#         self.name = name
#         self.selectable = selectable
#
#
# class DropView(ExecutableDDLElement):
#     """
#     Class that is used to drop a view. Every instance will be compiled into a ORM DROP VIEW statement.
#     """
#
#     def __init__(self, name: str):
#         self.name = name
#
#
# @compiles(CreateView)
# def _create_view(element: CreateView, compiler, **kw) -> str:
#     """
#     Compiles a CreateView instance into a SQL CREATE VIEW statement.
#     :param element: CreateView instance
#     :param compiler: compiler
#     :param kw: keyword arguments
#     :return: SQL CREATE VIEW statement
#     """
#
#     return "CREATE VIEW %s AS %s" % (
#         element.name,
#         compiler.sql_compiler.process(element.selectable, literal_binds=True),
#     )
#
#
# @compiles(DropView)
# def _drop_view(element: DropView, compiler, **kw) -> str:
#     """
#     Compiles a DropView instance into a SQL DROP VIEW statement.
#     :param element: DropView instance
#     :param compiler: compiler
#     :param kw: keyword arguments
#     :return: SQL DROP VIEW statement
#     """
#     return "DROP VIEW %s" % element.name
#
#
# def view_exists(ddl: Union[CreateView, DropView], target, connection: engine, **kw) -> bool:
#     """
#     Check if a view exists.
#     :param ddl: ddl instance
#     :param target: target object
#     :param connection: connection
#     :param kw: keyword arguments
#     :return: True if the view exists, False otherwise
#     """
#
#     return ddl.name in inspect(connection).get_view_names()
#
#
# def view_doesnt_exist(ddl: Union[CreateView, DropView], target, connection: engine, **kw) -> bool:
#     """
#     Check if a view does not exist.
#     :param ddl: ddl instance
#     :param target: target object
#     :param connection: connection
#     :param kw: keyword arguments
#     :return: True if the view does not exist, False otherwise
#     """
#
#     return not view_exists(ddl, target, connection, **kw)
#
#
# def view(name: str, metadata: MetaData, selectable: Select) -> TableClause:
#     """
#     Function used to control view creation and deletion. It will listen to the after_create and before_drop events
#     of the metadata object in order to either create or drop the view. The view needs to have a column id.
#     """
#     view = table(name)
#
#     view._columns._populate_separate_keys(
#         col._make_proxy(view) for col in selectable.selected_columns
#     )
#
#     event.listen(metadata, "after_create", CreateView(name, selectable).execute_if(callable_=view_doesnt_exist))
#     event.listen(metadata, "before_drop", DropView(name).execute_if(callable_=view_exists))
#
#     return view
#
#
# base = declarative_base(metadata=Base.metadata)
#
#
# class PickUpWithContextView(base):
#     """
#     View for pickup performables with context.
#     """
#
#     __robot_position: Position = sqlalchemy.orm.aliased(Position, flat=True)
#     """
#     3D Vector of robot position
#     """
#
#     __robot_pose: Pose = sqlalchemy.orm.aliased(Pose, flat=True)
#     """
#     Complete robot pose
#     """
#
#     __object_position: Position = sqlalchemy.orm.aliased(Position, flat=True)
#     """
#     3D Vector for object position
#     """
#
#     __table__ = view("PickUpWithContextView", Base.metadata,
#                      (select(PickUpAction.id, PickUpAction.arm, GraspDescription.approach_direction,
#                              GraspDescription.vertical_alignment, GraspDescription.rotate_gripper,
#                              RobotState.torso_height, (__robot_position.x-__object_position.x).label("relative_x"),
#                              (__robot_position.y-__object_position.y).label("relative_y"), Quaternion.x, Quaternion.y,
#                              Quaternion.z, Quaternion.w, Object.obj_type, TaskTreeNode.status)
#                       .join(TaskTreeNode.action.of_type(PickUpAction))
#                       .join(PickUpAction.robot_state)
#                       .join(__robot_pose, RobotState.pose)
#                       .join(__robot_position, __robot_pose.position)
#                       .join(Pose.orientation)
#                       .join(PickUpAction.object)
#                       .join(Object.pose)
#                       .join(__object_position, Pose.position)))
#
#     id: Mapped[int] = __table__.c.id
#     arm: Mapped[str] = __table__.c.arm
#     torso_height: Mapped[float] = __table__.c.torso_height
#     relative_x: Mapped[float] = column_property(__table__.c.relative_x)
#     relative_y: Mapped[float] = column_property(__table__.c.relative_y)
#     quaternion_x: Mapped[float] = __table__.c.x
#     quaternion_y: Mapped[float] = __table__.c.y
#     quaternion_z: Mapped[float] = __table__.c.z
#     quaternion_w: Mapped[float] = __table__.c.w
#     obj_type: Mapped[str] = __table__.c.obj_type
#     status: Mapped[str] = __table__.c.status
#
#
# class PlaceWithContextView(base):
#     """
#     View for pickup performables with context.
#     """
#
#     __robot_position: Position = sqlalchemy.orm.aliased(Position, flat=True)
#     """
#     3D Vector of robot position
#     """
#
#     __robot_pose: Pose = sqlalchemy.orm.aliased(Pose, flat=True)
#     """
#     Complete robot pose
#     """
#
#     __object_position: Position = sqlalchemy.orm.aliased(Position, flat=True)
#     """
#     3D Vector for object position
#     """
#
#     __table__ = view("PlaceWithContextView", Base.metadata,
#                      (select(PlaceAction.id, PlaceAction.arm, RobotState.torso_height,
#                              (__robot_position.x-__object_position.x).label("relative_x"),
#                              (__robot_position.y-__object_position.y).label("relative_y"), Quaternion.x, Quaternion.y,
#                              Quaternion.z, Quaternion.w, Object.obj_type, TaskTreeNode.status)
#                       .join(TaskTreeNode.action.of_type(PlaceAction))
#                       .join(PlaceAction.robot_state)
#                       .join(__robot_pose, RobotState.pose)
#                       .join(__robot_position, __robot_pose.position)
#                       .join(Pose.orientation)
#                       .join(PlaceAction.object)
#                       .join(Object.pose)
#                       .join(__object_position, Pose.position)))
#
#     id: Mapped[int] = __table__.c.id
#     arm: Mapped[str] = __table__.c.arm
#     torso_height: Mapped[float] = __table__.c.torso_height
#     relative_x: Mapped[float] = column_property(__table__.c.relative_x)
#     relative_y: Mapped[float] = column_property(__table__.c.relative_y)
#     quaternion_x: Mapped[float] = __table__.c.x
#     quaternion_y: Mapped[float] = __table__.c.y
#     quaternion_z: Mapped[float] = __table__.c.z
#     quaternion_w: Mapped[float] = __table__.c.w
#     obj_type: Mapped[str] = __table__.c.obj_type
#     status: Mapped[str] = __table__.c.status
