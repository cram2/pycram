from ..ros import get_ros_package_path

from ..datastructures.dataclasses import VirtualMobileBaseJoints
from ..datastructures.enums import (
    GripperState,
    Arms,
    Grasp,
    TorsoState,
    GripperType,
    StaticJointState,
)
from ..robot_description import (
    RobotDescription,
    KinematicChainDescription,
    EndEffectorDescription,
    RobotDescriptionManager,
    CameraDescription,
)
from ..helper import get_robot_description_path
from ..units import meter

filename = (
    get_ros_package_path("pycram") + "/resources/robots/" + "tiago_dual" + ".urdf"
)

mjcf_filename = get_robot_description_path("pal_robotics", "tiago_dual")

tiago_description = RobotDescription(
    "tiago_dual",
    "base_link",
    "torso_lift_link",
    "torso_lift_joint",
    filename,
    virtual_mobile_base_joints=VirtualMobileBaseJoints(),
    mjcf_path=mjcf_filename,
)

################################## Left Arm ##################################
left_arm = KinematicChainDescription(
    "left_arm",
    "torso_lift_link",
    "arm_left_7_link",
    tiago_description.urdf_object,
    arm_type=Arms.LEFT,
)

left_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "arm_left_1_joint": 0.27,
        "arm_left_2_joint": -1.07,
        "arm_left_3_joint": 1.5,
        "arm_left_4_joint": 1.96,
        "arm_left_5_joint": -2.0,
        "arm_left_6_joint": 1.2,
        "arm_left_7_joint": 0.5,
    },
)

tiago_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################
left_gripper = EndEffectorDescription(
    "left_gripper",
    "gripper_left_link",
    "gripper_left_grasping_frame",
    tiago_description.urdf_object,
)

left_gripper.add_static_joint_states(
    GripperState.OPEN,
    {"gripper_left_left_finger_joint": 0.048, "gripper_left_right_finger_joint": 0.048},
)

left_gripper.add_static_joint_states(
    GripperState.CLOSE,
    {"gripper_left_left_finger_joint": 0.0, "gripper_left_right_finger_joint": 0.0},
)
left_gripper.end_effector_type = GripperType.PARALLEL
left_gripper.opening_distance = 0.09 * meter  # measured
left_arm.end_effector = left_gripper

################################## Right Arm ##################################
right_arm = KinematicChainDescription(
    "right_arm",
    "torso_lift_link",
    "arm_right_7_link",
    tiago_description.urdf_object,
    arm_type=Arms.RIGHT,
)

right_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "arm_right_1_joint": 0.27,
        "arm_right_2_joint": -1.07,
        "arm_right_3_joint": 1.5,
        "arm_right_4_joint": 2.0,
        "arm_right_5_joint": -2.0,
        "arm_right_6_joint": 1.2,
        "arm_right_7_joint": 0.5,
    },
)

tiago_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################
right_gripper = EndEffectorDescription(
    "right_gripper",
    "gripper_right_link",
    "gripper_right_grasping_frame",
    tiago_description.urdf_object,
)

right_gripper.add_static_joint_states(
    GripperState.OPEN,
    {
        "gripper_right_left_finger_joint": 0.048,
        "gripper_right_right_finger_joint": 0.048,
    },
)

right_gripper.add_static_joint_states(
    GripperState.CLOSE,
    {"gripper_right_left_finger_joint": 0.0, "gripper_right_right_finger_joint": 0.0},
)
right_gripper.end_effector_type = GripperType.PARALLEL
right_gripper.opening_distance = 0.09 * meter  # measured
right_arm.end_effector = right_gripper

################################## Torso ##################################
torso = KinematicChainDescription(
    "torso", "torso_fixed_link", "torso_lift_link", tiago_description.urdf_object
)

torso.add_static_joint_states(TorsoState.HIGH, {"torso_lift_joint": 0.3})

torso.add_static_joint_states(TorsoState.MID, {"torso_lift_joint": 0.15})

torso.add_static_joint_states(TorsoState.LOW, {"torso_lift_joint": 0})

tiago_description.add_kinematic_chain_description(torso)

################################## Camera ##################################
camera = CameraDescription(
    "xtion_optical_frame", "xtion_optical_frame", 0.99483, 0.75049, 1.0665, 1.4165
)
tiago_description.add_camera_description(camera)

################################## Neck ##################################
tiago_description.add_kinematic_chain("neck", "torso_lift_link", "head_2_link")
tiago_description.set_neck(yaw_joint="head_1_joint", pitch_joint="head_2_joint")

################################# Grasps ##################################
front_grasp = [0, 0, 0, 1]
right_gripper.update_all_grasp_orientations(front_grasp)
left_gripper.update_all_grasp_orientations(front_grasp)


# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(tiago_description)
