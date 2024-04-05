from dataclasses import dataclass

import numpy as np
import sqlalchemy.orm
import sqlalchemy.sql
from sqlalchemy import select, Select
from typing_extensions import List

from ...world_concepts.costmaps import Rectangle, OccupancyCostmap
from ...designator import LocationDesignatorDescription
from ...designators.location_designator import CostmapLocation
from ...orm.action_designator import PickUpAction
from ...orm.base import RobotState, Quaternion
from ...orm.object_designator import Object
from ...orm.task import TaskTreeNode
from ...datastructures.pose import Pose
from ...orm.queries.queries import PickUpWithContext


@dataclass
class Location(LocationDesignatorDescription.Location):
    """
    A location that is described by a pose, a reachable arm, a torso height and a grasp.
    """
    pose: Pose
    reachable_arm: str
    torso_height: float
    grasp: str


class AbstractCostmapLocation(CostmapLocation):
    """
    Abstract Class for JPT and Database costmaps.
    """

    def __init__(self, target, reachable_for=None, reachable_arm=None):
        """
        Create a new AbstractCostmapLocation instance.
        :param target: The target object
        :param reachable_for:
        :param reachable_arm:
        """
        super().__init__(target, reachable_for, None, reachable_arm, None)

    def create_occupancy_rectangles(self) -> List[Rectangle]:
        """
        :return: A list of rectangles that represent the occupied space of the target object.
        """
        # create Occupancy costmap for the target object
        ocm = OccupancyCostmap(distance_to_obstacle=0.3, from_ros=False, size=200, resolution=0.02,
                               origin=self.target.pose)
        return ocm.partitioning_rectangles()


class DatabaseCostmapLocation(AbstractCostmapLocation):
    """
    Class that represents costmap locations from a given Database.
    The database has to have a schema that is compatible with the pycram.orm package.
    """

    def __init__(self, target, session: sqlalchemy.orm.Session = None, reachable_for=None, reachable_arm=None):
        """
        Create a Database Costmap

        :param target: The target object
        :param session: A session that can be used to execute queries
        :param reachable_for: The robot to grab the object with
        :param reachable_arm: The arm to use

        """
        super().__init__(target, reachable_for, reachable_arm)
        self.session = session

    @staticmethod
    def select_statement(query_context: PickUpWithContext) -> Select:
        return query_context.join_statement(select(PickUpAction.arm, PickUpAction.grasp, RobotState.torso_height,
                                                   query_context.relative_x, query_context.relative_y, Quaternion.x,
                                                   Quaternion.y, Quaternion.z, Quaternion.w).distinct())

    def create_query_from_occupancy_costmap(self) -> Select:
        """
        Create a query that queries all relative robot positions from an object that are not occluded using an
        OccupancyCostmap.
        """

        query_context = PickUpWithContext()

        # get query
        query = self.select_statement(query_context)

        # constraint query to correct object type and successful task status
        query = query.where(Object.type == self.target.type).where(TaskTreeNode.status == "SUCCEEDED")

        filters = []

        # for every rectangle
        for rectangle in self.create_occupancy_rectangles():
            # add sql filter
            filters.append(sqlalchemy.and_(query_context.relative_x >= rectangle.x_lower,
                                           query_context.relative_x < rectangle.x_upper,
                                           query_context.relative_y >= rectangle.y_lower,
                                           query_context.relative_y < rectangle.y_upper))

        return query.where(sqlalchemy.or_(*filters))

    def sample_to_location(self, sample: sqlalchemy.engine.row.Row) -> Location:
        """
        Convert a database row to a costmap location.

        :param sample: The database row.
        :return: The costmap location
        """

        target_x, target_y, target_z = self.target.pose.position_as_list()
        position = [target_x + sample[3], target_y + sample[4], 0]
        orientation = [sample[5], sample[6], sample[7], sample[8]]

        result = Location(Pose(position, orientation), sample.arm, sample.torso_height, sample.grasp)
        return result

    def __iter__(self) -> Location:
        statement = self.create_query_from_occupancy_costmap().limit(200)
        samples = self.session.execute(statement).all()
        if samples:
            for sample in samples:
                yield self.sample_to_location(sample)
        else:
            raise ValueError("No samples found")
