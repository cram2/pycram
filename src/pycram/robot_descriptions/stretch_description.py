import numpy as np
from types import MethodType
from ..tf_transformations import quaternion_from_euler

from ..pose_generator_and_validator import PoseGenerator
from ..ros import get_ros_package_path

from ..robot_description import (
    RobotDescription,
    KinematicChainDescription,
    EndEffectorDescription,
    CameraDescription,
    RobotDescriptionManager,
)
from ..datastructures.enums import (
    GripperState,
    Arms,
    Grasp,
    TorsoState,
    GripperType,
    StaticJointState,
)
from ..units import meter

filename = (
    get_ros_package_path("pycram")
    + "/resources/robots/"
    + "stretch_description"
    + ".urdf"
)

stretch_description = RobotDescription(
    "stretch_description", "base_link", "link_lift", "joint_lift", filename
)

################################## Right Arm ##################################
arm_description = KinematicChainDescription(
    "arm",
    "link_mast",
    "link_wrist_roll",
    stretch_description.urdf_object,
    arm_type=Arms.RIGHT,
)

arm_description.add_static_joint_states(
    StaticJointState.Park,
    {
        "joint_lift": 0.0,
        "joint_arm_l3": 0.0,
        "joint_arm_l2": 0.0,
        "joint_arm_l1": 0.0,
        "joint_arm_l0": 0.0,
        "joint_wrist_yaw": 0.0,
        "joint_wrist_pitch": 0.0,
        "joint_wrist_roll": 0.0,
    },
)

stretch_description.add_kinematic_chain_description(arm_description)

################################## Right Gripper ##################################
gripper_description = EndEffectorDescription(
    "arm", "link_straight_gripper", "link_grasp_center", stretch_description.urdf_object
)

gripper_description.add_static_joint_states(
    GripperState.OPEN,
    {"joint_gripper_finger_left": 0.59, "joint_gripper_finger_right": 0.59},
)
gripper_description.add_static_joint_states(
    GripperState.CLOSE,
    {"joint_gripper_finger_left": 0.0, "joint_gripper_finger_right": 0.0},
)
gripper_description.end_effector_type = GripperType.PARALLEL
gripper_description.opening_distance = 0.14 * meter  # assumption
arm_description.end_effector = gripper_description

################################## Neck ##################################
neck = KinematicChainDescription(
    "neck", "link_head", "link_head_tilt", stretch_description.urdf_object
)

stretch_description.add_kinematic_chain_description(neck)

stretch_description.set_neck(yaw_joint="joint_head_pan", pitch_joint="joint_head_tilt")

################################## Torso ##################################
torso = KinematicChainDescription(
    "torso", "link_mast", "link_lift", stretch_description.urdf_object
)

torso.add_static_joint_states(TorsoState.HIGH, {"joint_lift": 1})

torso.add_static_joint_states(TorsoState.MID, {"joint_lift": 0.5})

torso.add_static_joint_states(TorsoState.LOW, {"joint_lift": 0})

stretch_description.add_kinematic_chain_description(torso)

################################## Camera ##################################
realsense_color = CameraDescription(
    "camera_color_optical_frame", "camera_color_optical_frame", 1.322, 1.322
)
realsense_depth = CameraDescription(
    "camera_depth_optical_frame", "camera_depth_optical_frame", 1.307, 1.307
)
realsense_infra1 = CameraDescription(
    "camera_infra1_optical_frame", "camera_infra1_optical_frame", 1.307, 1.307
)
realsense_infra2 = CameraDescription(
    "camera_infra2_optical_frame", "camera_infra2_optical_frame", 1.257, 1.257
)

stretch_description.add_camera_description(realsense_color)
stretch_description.add_camera_description(realsense_depth)
stretch_description.add_camera_description(realsense_infra1)
stretch_description.add_camera_description(realsense_infra2)

################################## Grasps ##################################
gripper_description.update_all_grasp_orientations([0, 0, 0, 1])


################################### Custom Orientation Generator ##############
def stretch_orientation_generator(position, origin):
    angle = (
        np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x)
        + np.pi
        + np.pi / 16
    )
    quaternion = list(quaternion_from_euler(0, 0, angle + np.pi / 2, axes="sxyz"))
    return quaternion


################################ Load Function #################################
def load(description):
    RobotDescription.current_robot_description = description
    PoseGenerator.override_orientation_generator = stretch_orientation_generator


def unload(description):
    RobotDescription.current_robot_description = None
    PoseGenerator.override_orientation_generator = None


stretch_description.load = MethodType(load, stretch_description)
stretch_description.unload = MethodType(unload, stretch_description)

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(stretch_description)
