from types import MethodType

import numpy as np

from ..datastructures.dataclasses import VirtualMobileBaseJoints
from ..pose_generator_and_validator import PoseGenerator
from ..ros import get_ros_package_path
from ..robot_description import (
    RobotDescription,
    KinematicChainDescription,
    EndEffectorDescription,
    RobotDescriptionManager,
    CameraDescription,
)
from ..datastructures.enums import (
    Arms,
    Grasp,
    GripperState,
    TorsoState,
    GripperType,
    StaticJointState,
)
from ..tf_transformations import quaternion_from_euler
from ..units import meter

filename = (
    get_ros_package_path("pycram") + "/resources/robots/" + "iai_donbot" + ".urdf"
)

donbot_description = RobotDescription(
    "iai_donbot",
    "base_link",
    "ur5_base_link",
    "arm_base_mounting_joint",
    filename,
    virtual_mobile_base_joints=VirtualMobileBaseJoints(),
)

################################## Right Arm ##################################
left_arm = KinematicChainDescription(
    "left_arm",
    "ur5_base_link",
    "ur5_wrist_3_link",
    donbot_description.urdf_object,
    arm_type=Arms.LEFT,
)

left_arm.add_static_joint_states(
    "looking",
    {
        "ur5_shoulder_pan_joint": 0,
        "ur5_shoulder_lift_joint": -0.35,
        "ur5_elbow_joint": -2.15,
        "ur5_wrist_1_joint": -0.7,
        "ur5_wrist_2_joint": 1.57,
        "ur5_wrist_3_joint": -1.57,
    },
)

left_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "ur5_shoulder_pan_joint": 3.23,
        "ur5_shoulder_lift_joint": -1.51,
        "ur5_elbow_joint": -1.57,
        "ur5_wrist_1_joint": 0,
        "ur5_wrist_2_joint": 1.57,
        "ur5_wrist_3_joint": -1.65,
    },
)

donbot_description.add_kinematic_chain_description(left_arm)

################################## Right Gripper ##################################

left_gripper = EndEffectorDescription(
    "left_gripper",
    "gripper_base_link",
    "gripper_tool_frame",
    donbot_description.urdf_object,
)

left_gripper.add_static_joint_states(
    GripperState.OPEN,
    {"gripper_joint": 0.109, "gripper_base_gripper_left_joint": -0.055},
)
left_gripper.add_static_joint_states(
    GripperState.CLOSE,
    {"gripper_joint": 0.0065, "gripper_base_gripper_left_joint": -0.0027},
)
left_gripper.end_effector_type = GripperType.PARALLEL
left_gripper.opening_distance = 0.11 * meter  # 2x 55mm for WSG050
left_arm.end_effector = left_gripper

################################## Torso ##################################
torso = KinematicChainDescription(
    "torso",
    "base_footprint",
    "ur5_base_link",
    donbot_description.urdf_object,
    include_fixed_joints=True,
)

# fixed joint, so all states set to 0
torso.add_static_joint_states(TorsoState.HIGH, {"arm_base_mounting_joint": 0})

torso.add_static_joint_states(TorsoState.MID, {"arm_base_mounting_joint": 0})

torso.add_static_joint_states(TorsoState.LOW, {"arm_base_mounting_joint": 0})

donbot_description.add_kinematic_chain_description(torso)

################################## Camera ##################################
camera = CameraDescription("camera_link", "camera_link", 0.75049, 0.5, 1.2)
donbot_description.add_camera_description(camera)

################################## Neck ##################################
donbot_description.add_kinematic_chain("neck", "ur5_base_link", "ur5_base_link")

################################# Grasps ##################################
left_gripper.update_all_grasp_orientations([0.707, -0.707, 0.707, -0.707])


################################### Custom Orientation Generator ##############
def donbot_orientation_generator(position, origin):
    angle = (
        np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x)
        - np.pi / 2
    )

    quaternion = list(quaternion_from_euler(0, 0, angle, axes="sxyz"))
    return quaternion


################################ Load Function #################################
def load(description):
    RobotDescription.current_robot_description = description
    PoseGenerator.override_orientation_generator = donbot_orientation_generator


def unload(description):
    RobotDescription.current_robot_description = None
    PoseGenerator.override_orientation_generator = None


donbot_description.load = MethodType(load, donbot_description)
donbot_description.unload = MethodType(unload, donbot_description)

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(donbot_description)
