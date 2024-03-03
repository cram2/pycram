import numpy as np
import sqlalchemy.orm
import sqlalchemy.sql
from sqlalchemy import select, Select
import pycram.designators.location_designator
import pycram.task
from pycram.costmaps import OccupancyCostmap
from pycram.orm.action_designator import PickUpAction, Action
from pycram.orm.object_designator import Object
from pycram.orm.base import Position, RobotState, Pose as ORMPose, Quaternion
from pycram.orm.task import TaskTreeNode, Code
from .jpt_location import JPTCostmapLocation
from ...pose import Pose


class DatabaseCostmapLocation(pycram.designators.location_designator.CostmapLocation):
    """
    Class that represents costmap locations from a given Database.
    The database has to have a schema that is compatible with the pycram.orm package.
    """

    def __init__(self, target, session: sqlalchemy.orm.Session = None,
                 reachable_for=None, reachable_arm=None, resolver=None):
        """
        Create a Database Costmap

        :param target: The target object
        :param session: A session that can be used to execute queries
        :param reachable_for: The robot to grab the object with
        :param reachable_arm: The arm to use

        """
        super().__init__(target, reachable_for, None, reachable_arm, resolver)
        self.session = session

    def create_query_from_occupancy_costmap(self) -> Select:
        """
        Create a query that queries all relative robot positions from an object that are not occluded using an
        OccupancyCostmap.
        """

        robot_pos = sqlalchemy.orm.aliased(Position)
        robot_pose = sqlalchemy.orm.aliased(ORMPose)
        object_pos = sqlalchemy.orm.aliased(Position)
        relative_x = robot_pos.x - object_pos.x
        relative_y = robot_pos.y - object_pos.y

        # query all relative robot positions in regard to an objects position
        # make sure to order the joins() correctly
        query = (select(PickUpAction.arm, PickUpAction.grasp, RobotState.torso_height, relative_x, relative_y,
                        Quaternion.x, Quaternion.y, Quaternion.z, Quaternion.w).distinct()
                 .join(TaskTreeNode.code)
                 .join(Code.designator.of_type(PickUpAction))
                 .join(PickUpAction.robot_state)
                 .join(robot_pose, RobotState.pose)
                 .join(robot_pos, robot_pose.position)
                 .join(ORMPose.orientation)
                 .join(PickUpAction.object)
                 .join(Object.pose)
                 .join(object_pos, ORMPose.position).where(Object.type == self.target.type)
                                                    .where(TaskTreeNode.status == "SUCCEEDED"))

        # create Occupancy costmap for the target object

        ocm = OccupancyCostmap(distance_to_obstacle=0.3, from_ros=False, size=200, resolution=0.02,
                               origin=self.target.pose)
        ocm.visualize()

        # working on a copy of the costmap, since found rectangles are deleted
        map = np.copy(ocm.map)

        origin = np.array([ocm.height / 2, ocm.width / 2])

        filters = []

        for i in range(0, map.shape[0]):
            for j in range(0, map.shape[1]):
                if map[i][j] > 0:
                    curr_width = ocm._find_consectuive_line((i, j), map)
                    curr_pose = (i, j)
                    curr_height = ocm._find_max_box_height((i, j), curr_width, map)

                    x_lower = (curr_pose[0] - origin[0]) * ocm.resolution
                    x_upper = (curr_pose[0] + curr_width - origin[0]) * ocm.resolution
                    y_lower = (curr_pose[1] - origin[1]) * ocm.resolution
                    y_upper = (curr_pose[1] + curr_height - origin[1]) * ocm.resolution

                    map[i:i + curr_height, j:j + curr_width] = 0

                    filters.append(sqlalchemy.and_(relative_x >= x_lower, relative_x < x_upper,
                                                   relative_y >= y_lower, relative_y < y_upper))

        return query.where(sqlalchemy.or_(*filters))

    def sample_to_location(self, sample: sqlalchemy.engine.row.Row) -> JPTCostmapLocation.Location:
        """
        Convert a database row to a costmap location.

        :param sample: The database row.
        :return: The costmap location
        """
        target_x, target_y, target_z = self.target.pose.position_as_list()
        position = [target_x + sample[3], target_y + sample[4], 0]
        orientation = [sample[5], sample[6], sample[7], sample[8]]

        result = JPTCostmapLocation.Location(Pose(position, orientation), sample.arm, sample.torso_height, sample.grasp)
        return result

    def __iter__(self) -> JPTCostmapLocation.Location:
        statement = self.create_query_from_occupancy_costmap().limit(200)
        samples = self.session.execute(statement).all()
        if samples:
            for sample in samples:
                yield self.sample_to_location(sample)
        else:
            raise ValueError("No samples found")
