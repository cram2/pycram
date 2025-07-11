import numpy as np

from ..datastructures.dataclasses import VirtualMobileBaseJoints
from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import Arms, GripperState, GripperType, TorsoState, StaticJointState
from ..ros import  get_ros_package_path
from ..units import meter

from ..helper import get_robot_description_path

filename = get_ros_package_path('pycram') + '/resources/robots/' + "pr2" + '.urdf'

mjcf_filename = get_robot_description_path("", "pr2")

pr2_description = RobotDescription("pr2", "base_link", "torso_lift_link",
                                   "torso_lift_joint", filename,
                                   virtual_mobile_base_joints=VirtualMobileBaseJoints(),
                                   mjcf_path=mjcf_filename,
                                   ignore_joints=['torso_lift_motor_screw_joint', 'r_gripper_motor_slider_joint',
                                                  'r_gripper_motor_screw_joint', 'r_gripper_joint',
                                                  'l_gripper_motor_slider_joint', 'l_gripper_motor_screw_joint',
                                                  'l_gripper_joint'])

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left", "torso_lift_link", "l_wrist_roll_link",
                                     pr2_description.urdf_object, arm_type=Arms.LEFT)
left_arm.add_static_joint_states(StaticJointState.Park, {'l_shoulder_pan_joint': 1.712,
                                          'l_shoulder_lift_joint': -0.264,
                                          'l_upper_arm_roll_joint': 1.38,
                                          'l_elbow_flex_joint': -2.12,
                                          'l_forearm_roll_joint': 16.996+np.pi,
                                          'l_wrist_flex_joint': -0.073,
                                          'l_wrist_roll_joint': 0.0})
pr2_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################
left_gripper = EndEffectorDescription("left_gripper", "l_gripper_palm_link", "l_gripper_tool_frame",
                                      pr2_description.urdf_object,
                                      fingers_link_names=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link'])
left_gripper.add_static_joint_states(GripperState.OPEN, {'l_gripper_l_finger_joint': 0.548,
                                                         'l_gripper_r_finger_joint': 0.548})
left_gripper.add_static_joint_states(GripperState.CLOSE, {'l_gripper_l_finger_joint': 0.0,
                                                          'l_gripper_r_finger_joint': 0.0})
left_gripper.end_effector_type = GripperType.PARALLEL
left_gripper.opening_distance = 0.086 * meter
left_arm.end_effector = left_gripper

################################## Right Arm ##################################
right_arm = KinematicChainDescription("right", "torso_lift_link", "r_wrist_roll_link",
                                      pr2_description.urdf_object, arm_type=Arms.RIGHT)
right_arm.add_static_joint_states(StaticJointState.Park, {'r_shoulder_pan_joint': -1.712,
                                           'r_shoulder_lift_joint': -0.256,
                                           'r_upper_arm_roll_joint': -1.463,
                                           'r_elbow_flex_joint': -2.12,
                                           'r_forearm_roll_joint': 1.766+np.pi,
                                           'r_wrist_flex_joint': -0.07,
                                           'r_wrist_roll_joint': 0.051})
pr2_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################
right_gripper = EndEffectorDescription("right_gripper", "r_gripper_palm_link", "r_gripper_tool_frame",
                                       pr2_description.urdf_object,
                                       fingers_link_names=['r_gripper_l_finger_tip_link', 'r_gripper_r_finger_tip_link'])
right_gripper.add_static_joint_states(GripperState.OPEN, {'r_gripper_l_finger_joint': 0.548,
                                                          'r_gripper_r_finger_joint': 0.548})
right_gripper.add_static_joint_states(GripperState.CLOSE, {'r_gripper_l_finger_joint': 0.0,
                                                           'r_gripper_r_finger_joint': 0.0})
right_gripper.end_effector_type = GripperType.PARALLEL
right_gripper.opening_distance = 0.086 * meter
right_arm.end_effector = right_gripper

################################## Camera ##################################
camera = CameraDescription("kinect_camera", "wide_stereo_optical_frame", 1.27,
                           1.60, 0.99483, 0.75049)
pr2_description.add_camera_description(camera)

################################## Torso ##################################
torso = KinematicChainDescription("torso", "base_link", "torso_lift_link",
                                  pr2_description.urdf_object)

torso.add_static_joint_states(TorsoState.HIGH, {"torso_lift_joint": 0.3})

torso.add_static_joint_states(TorsoState.MID, {"torso_lift_joint": 0.15})

torso.add_static_joint_states(TorsoState.LOW, {"torso_lift_joint": 0})

pr2_description.add_kinematic_chain_description(torso)

################################## Neck ##################################
pr2_description.set_neck(yaw_joint="head_pan_joint", pitch_joint="head_tilt_joint")

################################# Grasps ##################################
left_gripper.update_all_grasp_orientations([0, 0, 0, 1])
left_gripper.set_approach_axis([1, 0, 0])
right_gripper.update_all_grasp_orientations([0, 0, 0, 1])
right_gripper.set_approach_axis([1, 0, 0])

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(pr2_description)
