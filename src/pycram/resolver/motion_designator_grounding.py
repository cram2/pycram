from pycram.designators.motion_designator import *
from pycram.bullet_world import BulletWorld
from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from pycram.designator import DesignatorError
import logging

"""
The grounding functions for the Motion designator descriptions in pycram/motion_designator.py
They all inferre missing properties and return a dictionaty with the properties as
value.
"""


def ground_move(description):
    if not description.orientation:
        # get_orientation returns tuple, the conversion is because of the type
        # check in _check_properties
        description.orientation = list(BulletWorld.robot.get_orientation())
    description._check_properties("[Motion Designator] Moving")
    return description.__dict__

def ground_pick_up(description):
    if not description.arm:
        description.arm = 'left'
    description.gripper = robot_description.i.get_tool_frame(description.arm)
    if not description.grasp:
        robot_description.i.grasps.get_grasps_for_object(description.object)[0]
    description._check_properties("[Motion Designator] Pick-Up")
    return description.__dict__

def ground_place(description):
    if not description.arm:
        description.arm = 'left'
    description.gripper = robot_description.i.get_tool_frame(description.arm)
    description._check_properties("[Motion Designator] Place")
    return description.__dict__

def ground_accessing(description):
    if not description.arm:
        description.arm = 'left'
    description.gripper = robot_description.i.get_tool_frame(description.arm)
    description._check_properties("[Motion Designator] Accessing")
    return description.__dict__

def ground_move_tcp(description):
    if not description.arm:
        description.arm = 'left'
    description._check_properties("[Motion Designator] Move-TCP")
    return description.__dict__

def ground_looking(description):
    if description.target and description.object:
        logging.warning(f"[Looking Designator Resolution] Target and Object parameter provided. Only Object will be used.")
        return {'cmd': description.cmd,
                'target': BulletWorld.current_bullet_world.get_objects_by_name(description.object)[0].get_pose()}
    if description.object:
        return {'cmd': description.cmd,
                'target': BulletWorld.current_bullet_world.get_objects_by_name(description.object)[0].get_pose()}
    if description.target:
        if len(description.target) == 3:
            description.target = [description.target, [0, 0, 0, 1]]
        return description.make_dictionary(["cmd", "target"])
    if not description.target and not description.object:
        description._check_properties("[Motion Designator] Looking")

def ground_move_gripper(description):
    description._check_properties("[Motion Designator] Move-gripper")
    return description.__dict__

def ground_detect(description):
    if not description.cam_frame:
        description.cam_frame = robot_description.i.get_camera_frame()
    if not description.front_facing_axis:
        description.front_facing_axis = robot_description.i.front_facing_axis
    description._check_properties("[Motion Designator] Detecting")
    return description.__dict__

def ground_move_arm(description):
    if description.left_arm_config or description.right_arm_config:
        description.left_arm_poses = description.left_arm_config
        description.right_arm_poses = description.right_arm_config
        return description.__dict__
    if description.right_arm_poses or description.left_arm_poses:
        return description.__dict__
    else:
        description._check_properties("[Motion Designator] Move-arm-joints")

def ground_world_state_detecting(description):
    description._check_properties("[Motion Designator] World-state-detecting")
    return description.__dict__

def ground_move_joints(description):
    description._check_properties("[Motion Designator] Move Joints")
    robot = BulletWorld.robot
    if len(description.names) != len(description.positions):
        raise DesignatorError("[Motion Designator][Move Joints] The length of names and positions does not match")
    for i in range(len(description.names)):
        lower, upper = robot.get_joint_limits(description.names[i])
        if description.positions[i] < lower or description.positions[i] > upper:
            raise DesignatorError(f"[Motion Designator][Move Joints] The given configuration for the Joint {description.names[i]} violates its limits")
    return description.__dict__


def call_ground(desig):
    type_to_function = {MoveMotionDescription : ground_move,
                        PickUpMotionDescription: ground_pick_up,
                        PlaceMotionDescription: ground_place,
                        AccessingMotionDescription: ground_accessing,
                        MoveTCPMotionDescription: ground_move_tcp,
                        LookingMotionDescription: ground_looking,
                        MoveGripperMotionDescription: ground_move_gripper,
                        DetectingMotionDescription: ground_detect,
                        MoveArmJointsMotionDescription: ground_move_arm,
                        WorldStateDetectingMotionDescription: ground_world_state_detecting,
                        MoveJointsMotion: ground_move_joints}

    ground_function = type_to_function[type(desig._description)]
    return ground_function(desig._description)

MotionDesignator.resolvers['grounding'] = call_ground
