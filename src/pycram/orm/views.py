from sqlalchemy.orm import declarative_base
from typing_extensions import Union
import sqlalchemy.orm
from sqlalchemy import table, inspect, event, select, engine, MetaData, Select, TableClause, ExecutableDDLElement
from sqlalchemy.ext.compiler import compiles
from .action_designator import PickUpAction
from .base import Position, RobotState, Pose, Base, Quaternion
from .object_designator import Object
from .tasktree import TaskTreeNode


class CreateView(ExecutableDDLElement):
    """
    Class that is used to create a view. Every instance will be compiled into a SQL CREATE VIEW statement.
    """

    def __init__(self, name: str, selectable: Select):
        self.name = name
        self.selectable = selectable


class DropView(ExecutableDDLElement):
    """
    Class that is used to drop a view. Every instance will be compiled into a SQL DROP VIEW statement.
    """

    def __init__(self, name: str):
        self.name = name


@compiles(CreateView)
def _create_view(element: CreateView, compiler, **kw) -> str:
    """
    Compiles a CreateView instance into a SQL CREATE VIEW statement.
    :param element: CreateView instance
    :param compiler: compiler
    :param kw: keyword arguments
    :return: SQL CREATE VIEW statement
    """

    return "CREATE VIEW %s AS %s" % (
        element.name,
        compiler.sql_compiler.process(element.selectable, literal_binds=True),
    )


@compiles(DropView)
def _drop_view(element: DropView, compiler, **kw) -> str:
    """
    Compiles a DropView instance into a SQL DROP VIEW statement.
    :param element: DropView instance
    :param compiler: compiler
    :param kw: keyword arguments
    :return: SQL DROP VIEW statement
    """
    return "DROP VIEW %s" % element.name


def view_exists(ddl: Union[CreateView, DropView], target, connection: engine, **kw) -> bool:
    """
    Check if a view exists.
    :param ddl: ddl instance
    :param target: target object
    :param connection: connection
    :param kw: keyword arguments
    :return: True if the view exists, False otherwise
    """

    return ddl.name in inspect(connection).get_view_names()


def view_doesnt_exist(ddl: Union[CreateView, DropView], target, connection: engine, **kw) -> bool:
    """
    Check if a view does not exist.
    :param ddl: ddl instance
    :param target: target object
    :param connection: connection
    :param kw: keyword arguments
    :return: True if the view does not exist, False otherwise
    """

    return not view_exists(ddl, target, connection, **kw)


def view(name: str, metadata: MetaData, selectable: Select) -> TableClause:
    """
    Function used to control view creation and deletion. It will listen to the after_create and before_drop events
    of the metadata object in order to either create or drop the view. The view needs to have a column id.
    """
    view = table(name)

    view._columns._populate_separate_keys(
        col._make_proxy(view) for col in selectable.selected_columns
    )

    event.listen(metadata, "after_create", CreateView(name, selectable).execute_if(callable_=view_doesnt_exist))
    event.listen(metadata, "before_drop", DropView(name).execute_if(callable_=view_exists))

    return view


base = declarative_base(metadata=Base.metadata)


class PickUpWithContextView(base):
    """
    View for pickup performables with context.
    """

    __robot_position: Position = sqlalchemy.orm.aliased(Position, flat=True)
    """
    3D Vector of robot position
    """

    __robot_pose: Pose = sqlalchemy.orm.aliased(Pose, flat=True)
    """
    Complete robot pose
    """

    __object_position: Position = sqlalchemy.orm.aliased(Position, flat=True)
    """
    3D Vector for object position
    """

    __relative_x = (__robot_position.x - __object_position.x)
    """
    Distance on x axis between robot and object
    """

    __relative_y = (__robot_position.y - __object_position.y)
    """
    Distance on y axis between robot and object
    """

    __table__ = view("PickUpWithContextView", Base.metadata,
                     (select(PickUpAction.id.label("id"), PickUpAction.arm.label("arm"),
                             PickUpAction.grasp.label("grasp"), RobotState.torso_height.label("torso_height"),
                             __relative_x.label("relative_x"), __relative_y.label("relative_y"),
                             Quaternion.x.label("quaternion_x"), Quaternion.y.label("quaternion_y"),
                             Quaternion.z.label("quaternion_z"), Quaternion.w.label("quaternion_w"),
                             Object.obj_type.label("obj_type"), TaskTreeNode.status.label("status"))
                      .join(TaskTreeNode.action.of_type(PickUpAction))
                      .join(PickUpAction.robot_state)
                      .join(__robot_pose, RobotState.pose)
                      .join(__robot_position, __robot_pose.position)
                      .join(Pose.orientation)
                      .join(PickUpAction.object)
                      .join(Object.pose)
                      .join(__object_position, Pose.position)))
