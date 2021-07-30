__all__ = ["ActionDesignator",
           "MoveTorsoActionDescription",
           "SetGripperActionDescription",
           "ReleaseActionDescription",
           "GripActionDescription",
           "MoveArmsInSequenceDescription",
           "MoveArmsIntoConfigurationDescription",
           "ParkArmsDescription",
           "PickUpDescription",
           "PlaceDescription",
           "NavigateDescription",
           "TransportObjectDescription",
           "LookAtActionDescription",
           "DetectActionDescription",
           "OpenActionDescription",
           "CloseActionDescription"]

from typing import List
from .task import with_tree
from .designator import Designator

class ActionDesignator: # (Designator):
    resolver = {}
    def __init__(self, description):
        self.description = description

    def reference(self):
        resolver = ActionDesignator.resolver[self.description.resolver]
        solution = resolver(self)
        return self


    def perform(self):
        #desc = self.description.ground()
        desc = self._reference()
        return desc.function()

    def __call__(self, *args, **kwargs):
        return self.perform()

class ActionDesignatorDescription:
    function = None

    def ground(self):
        return self

class MoveTorsoActionDescription(ActionDesignatorDescription):
    def __init__(self, position, resolver="grounding"):
        self.position = position
        self.resolver = resolver

class SetGripperActionDescription(ActionDesignatorDescription):
    def __init__(self, gripper, opening, resolver="grounding"):
        self.gripper = gripper
        self.opening = opening
        self.resolver = resolver

class ReleaseActionDescription(ActionDesignatorDescription):
    def __init__(self, gripper, object_designator=None, resolver="grounding"):
        self.gripper = gripper
        self.object_designator = object_designator
        self.resolver = resolver

class GripActionDescription(ActionDesignatorDescription):
    def __init__(self, gripper, object_designator=None, effort=None, resolver="grounding"):
        self.gripper = gripper
        self.object_designator = object_designator
        self.effort = effort
        self.grasped_object = None
        self.resolver = resolver

class MoveArmsIntoConfigurationDescription(ActionDesignatorDescription):
    def __init__(self, left_configuration=None, right_configuration=None, resolver="grounding"):
        self.left_configuration = left_configuration
        self.right_configuration = right_configuration
        self.left_joint_states = {}
        self.right_joint_states = {}
        self.resolver = resolver

class MoveArmsInSequenceDescription(ActionDesignatorDescription):
    def __init__(self, left_trajectory : List = [], right_trajectory : List = [], resolver="grounding"):
        self.left_trajectory = left_trajectory
        self.right_trajectory = right_trajectory
        self.resolver = resolver

class ParkArmsDescription(ActionDesignatorDescription):
    def __init__(self, arm, resolver="grounding"):
        self.arm = arm
        self.resolver = resolver

class PickUpDescription(ActionDesignatorDescription):
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

class PlaceDescription(ActionDesignatorDescription):
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

class NavigateDescription(ActionDesignatorDescription):
    def __init__(self, object_designator=None, target_location=None, target_position=None, target_orientation=None, resolver="grounding"):
        if object_designator and target_location:
            print("Warning: When providing both an object and a target location to navigate, only the object designator"
                  "will be considered.")
        self.object_designator = object_designator
        self.target_location = target_location
        self.target_position = target_position
        self.target_orientation = target_orientation
        self.resolver = resolver

class TransportObjectDescription(ActionDesignatorDescription):
    def __init__(self, object_designator, arm, target_location, resolver="grounding"):
        self.object_designator = object_designator
        self.arm = arm
        self.target_location = target_location
        self.resolver = resolver

class LookAtActionDescription(ActionDesignatorDescription):
    def __init__(self, target, resolver="grounding"):
        self.target = target
        self.resolver = resolver

class DetectActionDescription(ActionDesignatorDescription):
    def __init__(self, object_designator, resolver="grounding"):
        self.object_designator = object_designator
        self.resolver = resolver

class OpenActionDescription(ActionDesignatorDescription):
    def __init__(self, object_designator, arm, distance=None, resolver="grounding"):
        self.object_designator = object_designator
        self.arm = arm
        self.distance = distance
        self.resolver = resolver

class CloseActionDescription(ActionDesignatorDescription):
    def __init__(self, object_designator, arm, resolver="grounding"):
        self.object_designator = object_designator
        self.arm = arm
        self.resolver = resolver
