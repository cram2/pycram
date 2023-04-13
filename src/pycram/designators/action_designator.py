__all__ = ["ActionDesignator",
           "MoveTorsoAction",
           "SetGripperAction",
           "ReleaseAction",
           "GripAction",
           "MoveArmsInSequenceAction",
           "MoveArmsIntoConfigurationAction",
           "ParkArmsAction",
           "PickUpAction",
           "PlaceAction",
           "NavigateAction",
           "TransportAction",
           "LookAtAction",
           "DetectAction",
           "OpenAction",
           "CloseAction"]

from typing import List, Optional

import sqlalchemy.orm

from ..orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                     PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction)
from ..orm.base import Quaternion, Position, Base


class ActionDesignator:  # (Designator):
    resolver = {}

    def __init__(self, description):
        self.description = description

    def reference(self):
        resolver = ActionDesignator.resolver[self.description.resolver]
        solution = resolver(self)
        return solution

    def perform(self):
        # desc = self.description.ground()
        desc = self.reference()
        return desc.function()

    def __call__(self, *args, **kwargs):
        return self.perform()

    def to_sql(self) -> Base:
        raise NotImplementedError(f"{type(self)} has no implementation of to_sql.")

    def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
        raise NotImplementedError(f"{type(self)} has no implementation of insert.")


class ActionDesignatorDescription:
    function = None

    def ground(self):
        return self


class MoveTorsoAction(ActionDesignatorDescription):
    def __init__(self, position, resolver="grounding"):
        self.position: List[float] = position
        self.resolver = resolver


class SetGripperAction(ActionDesignatorDescription):
    def __init__(self, gripper, opening, resolver="grounding"):
        self.gripper: str = gripper
        self.opening: bool = opening
        self.resolver = resolver


# No resolving structure
class ReleaseAction(ActionDesignatorDescription):
    def __init__(self, gripper, object_designator=None, resolver="grounding"):
        self.gripper: str = gripper
        self.object_designator = object_designator
        self.resolver = resolver


# This Action can not be resolved, beacuse there is no resolving structure. And
# Just from the name it is not clear what it should do
class GripAction(ActionDesignatorDescription):
    def __init__(self, gripper, object_designator=None, effort=None, resolver="grounding"):
        self.gripper: str = gripper
        self.object_designator = object_designator
        self.effort: float = effort
        self.grasped_object = None
        self.resolver = resolver


# No Resolving Structure
class MoveArmsIntoConfigurationAction(ActionDesignatorDescription):
    def __init__(self, left_configuration=None, right_configuration=None, resolver="grounding"):
        self.left_configuration = left_configuration
        self.right_configuration = right_configuration
        self.left_joint_states = {}
        self.right_joint_states = {}
        self.resolver = resolver


# No Resolving structure
class MoveArmsInSequenceAction(ActionDesignatorDescription):
    def __init__(self, left_trajectory: List = [], right_trajectory: List = [], resolver="grounding"):
        self.left_trajectory = left_trajectory
        self.right_trajectory = right_trajectory
        self.resolver = resolver


class ParkArmsAction(ActionDesignatorDescription):
    def __init__(self, arm, resolver="grounding"):
        self.arm = arm
        self.resolver = resolver

    def to_sql(self) -> ORMParkArmsAction:
        return ORMParkArmsAction(self.arm.name)
        # return orm.action_designator.ParkArmsAction(self.arm.name)

    def insert(self, session: sqlalchemy.orm.session.Session) -> ORMParkArmsAction:
        action = self.to_sql()
        session.add(action)
        session.commit()
        return action


class PickUpAction(ActionDesignatorDescription):
    """
    Class representing picking something up.

    :ivar object_designator: The object designator that describes what to pick up.
    :ivar arm: The arm to use as string.
    :ivar grasp: The direction to grasp from.
    :ivar gripper_opening: Float describing how far the gripper should open, optional
    """

    def __init__(self, object_designator, arm=None, grasp=None, resolver="grounding"):
        self.object_designator = object_designator
        self.arm: str = arm
        self.grasp: str = grasp
        self.resolver = resolver

        # Grounded attributes
        self.gripper_opening: Optional[float] = None
        # self.effort = None
        # self.left_reach_poses = []
        # self.right_reach_poses = []
        # self.left_grasp_poses = []
        # self.right_grasp_poses = []
        # self.left_lift_poses = []
        # self.right_lift_poses = []

    def to_sql(self) -> ORMPickUpAction:
        return ORMPickUpAction(self.arm, self.grasp, self.gripper_opening)

    def insert(self, session: sqlalchemy.orm.session.Session):
        action = self.to_sql()

        # try to create the object designator
        if self.object_designator:
            # TODO discuss why it is _description
            od = self.object_designator._description.insert(session)
            action.object = od.id
        else:
            action.object = None

        session.add(action)
        session.commit()

        return action


class PlaceAction(ActionDesignatorDescription):
    def __init__(self, object_designator, target_location, arm=None, resolver="grounding"):
        self.object_designator = object_designator
        self.target_location = target_location
        self.arm: str = arm
        self.resolver = resolver

        # Grounded attributes
        # self.left_reach_poses = []
        # self.right_reach_poses = []
        # self.left_place_poses = []
        # self.right_place_poses = []
        # self.left_retract_poses = []
        # self.right_retract_poses = []

    def to_sql(self) -> ORMPlaceAction:
        return ORMPlaceAction(self.arm)

    def insert(self, session) -> ORMPlaceAction:
        action = self.to_sql()

        if self.object_designator:
            # TODO discuss why it is _description
            od = self.object_designator._description.insert(session)
            action.object = od.id
        else:
            action.object = None

        if self.target_location:
            position = Position(*self.target_location[0])
            orientation = Quaternion(*self.target_location[1])
            session.add(position)
            session.add(orientation)
            session.commit()
            action.position = position.id
            action.orientation = orientation.id
        else:
            action.position = None
            action.orientation = None

        session.add(action)
        session.commit()
        return action


class NavigateAction(ActionDesignatorDescription):
    def __init__(self, target_position, target_orientation=None, resolver="grounding"):
        self.target_position = target_position
        self.target_orientation = target_orientation
        self.resolver = resolver

    def to_sql(self) -> ORMNavigateAction:
        return ORMNavigateAction()

    def insert(self, session) -> ORMNavigateAction:
        # initialize position and orientation
        position = Position(*self.target_position)
        orientation = Quaternion(*self.target_orientation)

        # add those to the database and get the primary keys
        session.add(position)
        session.add(orientation)
        session.commit()

        # create the navigate action orm object
        navigate_action = self.to_sql()

        # set foreign keys
        navigate_action.position = position.id
        navigate_action.orientation = orientation.id

        # add it to the db
        session.add(navigate_action)
        session.commit()

        return navigate_action


class TransportAction(ActionDesignatorDescription):
    def __init__(self, object_designator, arm, target_location, resolver="grounding"):
        self.object_designator = object_designator
        self.arm = arm
        self.target_location = target_location  # orientation + location
        self.resolver = resolver


class LookAtAction(ActionDesignatorDescription):
    def __init__(self, target, resolver="grounding"):
        self.target = target
        self.resolver = resolver


class DetectAction(ActionDesignatorDescription):
    def __init__(self, object_designator, resolver="grounding"):
        self.object_designator = object_designator
        self.resolver = resolver


class OpenAction(ActionDesignatorDescription):
    def __init__(self, object_designator, arm, distance=None, resolver="grounding"):
        self.object_designator = object_designator
        self.arm = arm
        self.distance = distance
        self.resolver = resolver


class CloseAction(ActionDesignatorDescription):
    def __init__(self, object_designator, arm, resolver="grounding"):
        self.object_designator = object_designator
        self.arm = arm
        self.resolver = resolver
