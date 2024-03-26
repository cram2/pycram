import sqlalchemy.orm
from sqlalchemy import Select, alias
from ..task import TaskTreeNode
from ..action_designator import PickUpAction
from ..base import Position, RobotState, Pose
from ..object_designator import Object


class Query:
    """
    Abstract class for queries
    """


class PickUpWithContext(Query):
    """
    Query for pickup actions with context.
    """

    robot_position = sqlalchemy.orm.aliased(Position)
    """
    3D Vector of robot position
    """

    robot_pose = sqlalchemy.orm.aliased(Pose)
    """
    Complete robot pose
    """

    object_position = sqlalchemy.orm.aliased(Position)
    """
    3D Vector for object position
    """

    relative_x = (robot_position.x - object_position.x).label("relative_x")
    """
    Distance on x axis between robot and object
    """

    relative_y = (robot_position.y - object_position.y).label("relative_y")
    """
    Distance on y axis between robot and object
    """

    def join_statement(self, query: Select):
        return (query.join(TaskTreeNode).join(TaskTreeNode.action.of_type(PickUpAction))
                .join(PickUpAction.robot_state).join(self.robot_pose, RobotState.pose)
                .join(self.robot_position, self.robot_pose.position)
                .join(Pose.orientation)
                .join(PickUpAction.object)
                .join(Object.pose).join(self.object_position, Pose.position))
