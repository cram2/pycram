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

class ActionDesignator: # (Designator):
    def __init__(self, description):
        self.description = description

    def _reference(self):
        return self


    def perform(self):
        desc = self.description.ground()
        return desc.function()

    def __call__(self, *args, **kwargs):
        return self.perform()

class ActionDesignatorDescription:
    function = None

    def ground(self):
        return self

class MoveTorsoActionDescription(ActionDesignatorDescription):
    def __init__(self, position):
        self.position = position

class SetGripperActionDescription(ActionDesignatorDescription):
    def __init__(self, gripper, opening):
        self.gripper = gripper
        self.opening = opening

class ReleaseActionDescription(ActionDesignatorDescription):
    def __init__(self, gripper, object_designator=None):
        self.gripper = gripper
        self.object_designator = object_designator

class GripActionDescription(ActionDesignatorDescription):
    def __init__(self, gripper, object_designator=None, effort=None):
        self.gripper = gripper
        self.object_designator = object_designator
        self.effort = effort
        self.grasped_object = None

class MoveArmsIntoConfigurationDescription(ActionDesignatorDescription):
    def __init__(self, left_configuration=None, right_configuration=None):
        self.left_configuration = left_configuration
        self.right_configuration = right_configuration
        self.left_joint_states = {}
        self.right_joint_states = {}

class MoveArmsInSequenceDescription(ActionDesignatorDescription):
    def __init__(self, left_trajectory : List = [], right_trajectory : List = []):
        self.left_trajectory = left_trajectory
        self.right_trajectory = right_trajectory

class ParkArmsDescription(ActionDesignatorDescription):
    def __init__(self, arm):
        self.arm = arm

class PickUpDescription(ActionDesignatorDescription):
    def __init__(self, object_designator, arm=None, grasp=None):
        self.object_designator = object_designator
        self.arm = arm
        self.grasp = grasp

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
    def __init__(self, object_designator, target_location, arm=None):
        self.object_designator = object_designator
        self.target_location = target_location
        self.arm = arm

        # Grounded attributes
        # self.left_reach_poses = []
        # self.right_reach_poses = []
        # self.left_place_poses = []
        # self.right_place_poses = []
        # self.left_retract_poses = []
        # self.right_retract_poses = []

class NavigateDescription(ActionDesignatorDescription):
    def __init__(self, object_designator=None, target_location=None, target_position=None, target_orientation=None):
        if object_designator and target_location:
            print("Warning: When providing both an object and a target location to navigate, only the object designator"
                  "will be considered.")
        self.object_designator = object_designator
        self.target_location = target_location
        self.target_position = target_position
        self.target_orientation = target_orientation

class TransportObjectDescription(ActionDesignatorDescription):
    def __init__(self, object_designator, arm, target_location):
        self.object_designator = object_designator
        self.arm = arm
        self.target_location = target_location

class LookAtActionDescription(ActionDesignatorDescription):
    def __init__(self, target):
        self.target = target

class DetectActionDescription(ActionDesignatorDescription):
    def __init__(self, object_designator):
        self.object_designator = object_designator

class OpenActionDescription(ActionDesignatorDescription):
    def __init__(self, object_designator, arm, distance=None):
        self.object_designator = object_designator
        self.arm = arm
        self.distance = distance

class CloseActionDescription(ActionDesignatorDescription):
    def __init__(self, object_designator, arm):
        self.object_designator = object_designator
        self.arm = arm
