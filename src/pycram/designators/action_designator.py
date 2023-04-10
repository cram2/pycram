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

from typing import List
import sqlalchemy.orm
import pycram.orm.action_designator
from pycram.orm.base import Base


class ActionDesignator: # (Designator):
    resolver = {}

    def __init__(self, description):
        self.description = description

    def reference(self):
        resolver = ActionDesignator.resolver[self.description.resolver]
        solution = resolver(self)
        return solution

    def perform(self):
        #desc = self.description.ground()
        desc = self.reference()
        return desc.function()

    def __call__(self, *args, **kwargs):
        return self.perform()

    def to_sql(self) -> pycram.orm.base.Base:
        raise NotImplementedError(f"{type(self)} has no implementation of to_sql.")

    def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> pycram.orm.base.Base:
        raise NotImplementedError(f"{type(self)} has no implementation of insert.")


class ActionDesignatorDescription:
    function = None

    def ground(self):
        return self


class MoveTorsoAction(ActionDesignatorDescription):
    def __init__(self, position, resolver="grounding"):
        self.position = position
        self.resolver = resolver


class SetGripperAction(ActionDesignatorDescription):
    def __init__(self, gripper, opening, resolver="grounding"):
        self.gripper = gripper
        self.opening = opening
        self.resolver = resolver


# No resolving structure
class ReleaseAction(ActionDesignatorDescription):
    def __init__(self, gripper, object_designator=None, resolver="grounding"):
        self.gripper = gripper
        self.object_designator = object_designator
        self.resolver = resolver


# This Action can not be resolved, beacuse there is no resolving structure. And
# Just from the name it is not clear what it should do
class GripAction(ActionDesignatorDescription):
    def __init__(self, gripper, object_designator=None, effort=None, resolver="grounding"):
        self.gripper = gripper
        self.object_designator = object_designator
        self.effort = effort
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
    def __init__(self, left_trajectory : List = [], right_trajectory : List = [], resolver="grounding"):
        self.left_trajectory = left_trajectory
        self.right_trajectory = right_trajectory
        self.resolver = resolver


class ParkArmsAction(ActionDesignatorDescription):
    def __init__(self, arm, resolver="grounding"):
        self.arm = arm
        self.resolver = resolver

    def to_sql(self) -> pycram.orm.action_designator.ParkArmsAction:
        return pycram.orm.action_designator.ParkArmsAction(self.arm.name)

    def insert(self, session: sqlalchemy.orm.session.Session) -> pycram.orm.action_designator.ParkArmsAction:
        action = self.to_sql()
        session.add(action)
        session.commit()
        return action


class PickUpAction(ActionDesignatorDescription):
    def __init__(self, object_designator, arm=None, grasp=None, resolver="grounding"):
        self.object_designator = object_designator
        self.arm = arm
        self.grasp = grasp
        self.resolver = resolver

        # Grounded attributes
        self.gripper_opening = None
        # self.effort = None
        # self.left_reach_poses = []
        # self.right_reach_poses = []
        # self.left_grasp_poses = []
        # self.right_grasp_poses = []
        # self.left_lift_poses = []
        # self.right_lift_poses = []


class PlaceAction(ActionDesignatorDescription):
    def __init__(self, object_designator, target_location, arm=None, resolver="grounding"):
        self.object_designator = object_designator
        self.target_location = target_location
        self.arm = arm
        self.resolver = resolver

        # Grounded attributes
        # self.left_reach_poses = []
        # self.right_reach_poses = []
        # self.left_place_poses = []
        # self.right_place_poses = []
        # self.left_retract_poses = []
        # self.right_retract_poses = []


class NavigateAction(ActionDesignatorDescription):
    def __init__(self, target_position, target_orientation=None, resolver="grounding"):
        self.target_position = target_position
        self.target_orientation = target_orientation
        self.resolver = resolver

    def to_sql(self) -> pycram.orm.action_designator.NavigateAction:
        return pycram.orm.action_designator.NavigateAction()

    def insert(self, session) -> pycram.orm.action_designator.NavigateAction:

        # initialize position and orientation
        position = pycram.orm.base.Position(*self.target_position)
        orientation = pycram.orm.base.Quaternion(*self.target_orientation)

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
        self.target_location = target_location
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
