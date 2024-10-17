import os

from ..datastructures.enums import Arms, GripperState
from ..helper import get_robot_mjcf_path, find_multiverse_resources_path
from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager

multiverse_resources = find_multiverse_resources_path()
robot_relative_dir = "universal_robot"
robot_name = "ur5e"
filename = os.path.join(multiverse_resources, 'robots', robot_relative_dir, 'ur5e_with_gripper', 'urdf/ur5e.urdf')
mjcf_filename = get_robot_mjcf_path(robot_relative_dir, 'ur5e')

ur5_description = RobotDescription(robot_name, "base_link", "", "",
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
gripper = EndEffectorDescription("gripper", "gripper_2F_85", "right_pad",
                                 ur5_description.urdf_object)

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
