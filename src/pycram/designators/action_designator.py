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

class ReleaseAction(ActionDesignatorDescription):
    def __init__(self, gripper, object_designator=None, resolver="grounding"):
        self.gripper = gripper
        self.object_designator = object_designator
        self.resolver = resolver

class GripAction(ActionDesignatorDescription):
    def __init__(self, gripper, object_designator=None, effort=None, resolver="grounding"):
        self.gripper = gripper
        self.object_designator = object_designator
        self.effort = effort
        self.grasped_object = None
        self.resolver = resolver

class MoveArmsIntoConfigurationAction(ActionDesignatorDescription):
    def __init__(self, left_configuration=None, right_configuration=None, resolver="grounding"):
        self.left_configuration = left_configuration
        self.right_configuration = right_configuration
        self.left_joint_states = {}
        self.right_joint_states = {}
        self.resolver = resolver

class MoveArmsInSequenceAction(ActionDesignatorDescription):
    def __init__(self, left_trajectory : List = [], right_trajectory : List = [], resolver="grounding"):
        self.left_trajectory = left_trajectory
        self.right_trajectory = right_trajectory
        self.resolver = resolver

class ParkArmsAction(ActionDesignatorDescription):
    def __init__(self, arm, resolver="grounding"):
        self.arm = arm
        self.resolver = resolver

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
    def __init__(self, object_designator=None, target_location=None, target_position=None, target_orientation=None, resolver="grounding"):
        if object_designator and target_location:
            print("Warning: When providing both an object and a target location to navigate, only the object designator"
                  "will be considered.")
        self.object_designator = object_designator
        self.target_location = target_location
        self.target_position = target_position
        self.target_orientation = target_orientation
        self.resolver = resolver

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
