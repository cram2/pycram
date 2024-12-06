import os

from ..datastructures.enums import Arms, GripperState
from ..helper import get_robot_mjcf_path, find_multiverse_resources_path, get_robot_urdf_path_from_multiverse
from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager
from ..object_descriptors.urdf import ObjectDescription as URDFObject
from ..ros.logging import logwarn

multiverse_resources = find_multiverse_resources_path()
ROBOT_NAME = "ur5e"
GRIPPER_NAME = "gripper-2F-85"
GRIPPER_CMD_TOPIC = "/gripper_command"
OPEN_VALUE = 0.0
CLOSE_VALUE = 255.0
if multiverse_resources is None:
    logwarn("Could not initialize ur5e description as Multiverse resources path not found.")
else:
    robot_relative_dir = "universal_robot"
    filename = get_robot_urdf_path_from_multiverse(robot_relative_dir, ROBOT_NAME, resources_dir=multiverse_resources)
    mjcf_filename = get_robot_mjcf_path(robot_relative_dir, ROBOT_NAME, multiverse_resources=multiverse_resources)
    ur5_description = RobotDescription(ROBOT_NAME, "base_link", "", "",
                                       filename, mjcf_path=mjcf_filename)

    ################################## Arm ##################################
    arm = KinematicChainDescription("manipulator", "base_link", "wrist_3_link",
                                    ur5_description.urdf_object, arm_type=Arms.RIGHT)

    arm.add_static_joint_states("home", {'shoulder_pan_joint': 3.14,
                                         'shoulder_lift_joint': -1.56,
                                         'elbow_joint': 1.58,
                                         'wrist_1_joint': -1.57,
                                         'wrist_2_joint': -1.57,
                                         'wrist_3_joint': 0.0})

    ur5_description.add_kinematic_chain_description(arm)

    ################################## Gripper ##################################
    gripper_relative_dir = "robotiq"
    gripper_filename = get_robot_urdf_path_from_multiverse(gripper_relative_dir, GRIPPER_NAME,
                                                           resources_dir=multiverse_resources)
    gripper_urdf_obj = URDFObject(gripper_filename)
    gripper = EndEffectorDescription("gripper", GRIPPER_NAME, "right_pad",
                                     gripper_urdf_obj, gripper_object_name=GRIPPER_NAME)

    gripper.add_static_joint_states(GripperState.OPEN, {'right_driver_joint': 0.0,
                                                        'right_coupler_joint': 0.0,
                                                        'right_spring_link_joint': 0.0,
                                                        'right_follower_joint': 0.0,
                                                        'left_driver_joint': 0.0,
                                                        'left_coupler_joint': 0.0,
                                                        'left_spring_link_joint': 0.0,
                                                        'left_follower_joint': 0.0})
    gripper.add_static_joint_states(GripperState.CLOSE, {'right_driver_joint': 0.798,
                                                         'right_coupler_joint': 0.00366,
                                                         'right_spring_link_joint': 0.796,
                                                         'right_follower_joint': -0.793,
                                                         'left_driver_joint': 0.798,
                                                         'left_coupler_joint': 0.00366,
                                                         'left_spring_link_joint': 0.796,
                                                         'left_follower_joint': -0.793})

    arm.end_effector = gripper

    # Add to RobotDescriptionManager
    rdm = RobotDescriptionManager()
    rdm.register_description(ur5_description)
