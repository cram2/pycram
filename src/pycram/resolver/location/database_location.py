import numpy as np
import sqlalchemy.orm
import sqlalchemy.sql
from sqlalchemy import select, Select
from tf import transformations
import pycram.designators.location_designator
import pycram.task
from pycram.costmaps import OccupancyCostmap
from pycram.orm.action_designator import PickUpAction, Action
from pycram.orm.object_designator import Object
from pycram.orm.base import Position, RobotState, Designator, Base
from pycram.orm.task import TaskTreeNode, Code
from .jpt_location import JPTCostmapLocation
from ... import orm
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

        # query all relative robot positions in regard to an objects position
        # make sure to order the joins() correctly
        query = (select(PickUpAction.arm, PickUpAction.grasp, RobotState.torso_height, Position.x, Position.y)
                 .join(TaskTreeNode.code)
                 .join(Code.designator.of_type(PickUpAction))
                 .join(PickUpAction.robot_state)
                 .join(RobotState.pose)
                 .join(orm.base.Pose.position)
                 .join(PickUpAction.object).where(Object.type == self.target.type)
                                           .where(TaskTreeNode.status == "SUCCEEDED"))

        # create Occupancy costmap for the target object
        position, orientation = self.target.pose.to_list()
        position = list(position)
        position[-1] = 0

        ocm = OccupancyCostmap(distance_to_obstacle=0.3, from_ros=False, size=200, resolution=0.02,
                               origin=Pose(position, orientation))
        # ocm.visualize()

        # working on a copy of the costmap, since found rectangles are deleted
        map = np.copy(ocm.map)

        origin = np.array([ocm.height / 2, ocm.width / 2])

        filters = []

        # for every index pair (i, j) in the occupancy map
        for i in range(0, map.shape[0]):
            for j in range(0, map.shape[1]):

                # if this index has not been used yet
                if map[i][j] > 0:
                    # get consecutive box
                    width = ocm._find_consectuive_line((i, j), map)
                    height = ocm._find_max_box_height((i, j), width, map)

                    # mark box as used
                    map[i:i + height, j:j + width] = 0

                    # calculate to coordinates relative to the objects pose
                    pose = np.array([i, j])
                    lower_corner = (pose - origin) * ocm.resolution
                    upper_corner = (pose - origin + np.array([height, width])) * ocm.resolution
                    rectangle = np.array([lower_corner, upper_corner]).T

                    # transform to jpt query
                    filters.append(sqlalchemy.and_(robot_pos.x >= rectangle[0][0], robot_pos.x < rectangle[0][1],
                                                   robot_pos.y >= rectangle[1][0], robot_pos.y < rectangle[1][1]))
                    # query = self.model.bind({"x": list(rectangle[0]), "y": list(rectangle[1])})

        return query.where(sqlalchemy.or_(*filters))

    def sample_to_location(self, sample: sqlalchemy.engine.row.Row) -> JPTCostmapLocation.Location:
        """
        Convert a database row to a costmap location.

        :param sample: The database row.
        :return: The costmap location
        """
        target_x, target_y, target_z = self.target.pose.position_as_list()
        pose = [target_x + sample.x, target_y + sample.y, 0]
        angle = np.arctan2(pose[1] - target_y, pose[0] - target_x) + np.pi
        orientation = list(transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))

        result = JPTCostmapLocation.Location(Pose(pose, orientation), sample.arm, sample.torso_height, sample.grasp)
        return result

    def __iter__(self) -> JPTCostmapLocation.Location:
        statement = self.create_query_from_occupancy_costmap().limit(200)
        samples = self.session.execute(statement).all()
        for sample in samples:
            yield self.sample_to_location(sample)
