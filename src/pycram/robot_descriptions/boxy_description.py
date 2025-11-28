from ..ros import get_ros_package_path
from ..robot_description import (
    RobotDescription,
    CameraDescription,
    KinematicChainDescription,
    EndEffectorDescription,
    RobotDescriptionManager,
)
from ..datastructures.enums import (
    Arms,
    Grasp,
    GripperState,
    TorsoState,
    GripperType,
    StaticJointState,
)
from ..units import meter
from ..datastructures.dataclasses import VirtualMobileBaseJoints

filename = get_ros_package_path("pycram") + "/resources/robots/" + "boxy" + ".urdf"

boxy_description = RobotDescription(
    "boxy",
    "base_link",
    "triangle_base_link",
    "triangle_base_joint",
    filename,
    virtual_mobile_base_joints=VirtualMobileBaseJoints(),
)

################################## Right Arm ##################################
right_arm = KinematicChainDescription(
    "right_arm",
    "calib_right_arm_base_link",
    "right_arm_7_link",
    boxy_description.urdf_object,
    arm_type=Arms.RIGHT,
)

right_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "right_arm_0_joint": 1.858,
        "right_arm_1_joint": -0.70571,
        "right_arm_2_joint": -0.9614,
        "right_arm_3_joint": 0.602,
        "right_arm_4_joint": 2.5922,
        "right_arm_5_joint": 1.94065,
        "right_arm_6_joint": 1.28735,
    },
)

boxy_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################

right_gripper = EndEffectorDescription(
    "right_gripper",
    "right_gripper_base_link",
    "right_gripper_tool_frame",
    boxy_description.urdf_object,
)

right_gripper.add_static_joint_states(GripperState.OPEN, {"right_gripper_joint": 0.548})
right_gripper.add_static_joint_states(GripperState.CLOSE, {"right_gripper_joint": 0.0})
right_gripper.end_effector_type = (
    GripperType.PARALLEL
)  # current gripper in sim, change later
right_gripper.opening_distance = 0.11 * meter  # 2x 55mm for WSG050
right_arm.end_effector = right_gripper

################################## Left Arm ##################################
left_arm = KinematicChainDescription(
    "left_arm",
    "calib_left_arm_base_link",
    "left_arm_7_link",
    boxy_description.urdf_object,
    arm_type=Arms.LEFT,
)

left_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "left_arm_0_joint": -1.858,
        "left_arm_1_joint": 0.70571,
        "left_arm_2_joint": 0.9614,
        "left_arm_3_joint": -0.602,
        "left_arm_4_joint": -2.5922,
        "left_arm_5_joint": -1.94065,
        "left_arm_6_joint": -1.28735,
    },
)

boxy_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################

left_gripper = EndEffectorDescription(
    "left_gripper",
    "left_gripper_base_link",
    "left_gripper_tool_frame",
    boxy_description.urdf_object,
)

left_gripper.add_static_joint_states(GripperState.OPEN, {"left_gripper_joint": 0.548})
left_gripper.add_static_joint_states(GripperState.CLOSE, {"left_gripper_joint": 0.0})
left_gripper.end_effector_type = (
    GripperType.PARALLEL
)  # current gripper in sim, change later
left_gripper.opening_distance = 0.11 * meter  # 2x 55mm for WSG050
left_arm.end_effector = left_gripper

################################## Torso ##################################
torso = KinematicChainDescription(
    "torso", "base_link", "triangle_base_link", boxy_description.urdf_object
)

torso.add_static_joint_states(TorsoState.HIGH, {"triangle_base_joint": 0})

torso.add_static_joint_states(TorsoState.MID, {"triangle_base_joint": 0.29})

torso.add_static_joint_states(TorsoState.LOW, {"triangle_base_joint": -0.58})

boxy_description.add_kinematic_chain_description(torso)

################################## Camera ##################################
camera = CameraDescription(
    "head_mount_kinect2_rgb_optical_frame",
    "head_mount_kinect2_rgb_optical_frame",
    2.5,
    0.99483,
    0.75049,
)
boxy_description.add_camera_description(camera)

################################## Neck ##################################
boxy_description.add_kinematic_chain("neck", "neck_base_link", "neck_wrist_3_link")

################################# Grasps ##################################
left_gripper.update_all_grasp_orientations([1, 0, 1, 0])
right_gripper.update_all_grasp_orientations([1, 0, 1, 0])

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(boxy_description)
