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
    GripperType,
    TorsoState,
    StaticJointState,
)
from ..ros import get_ros_package_path
from ..datastructures.dataclasses import VirtualMobileBaseJoints

filename = get_ros_package_path("pycram") + "/resources/robots/" + "Armar6" + ".urdf"

armar_description = RobotDescription(
    "Armar6",
    "world",
    "torso",
    "torso_joint",
    filename,
    virtual_mobile_base_joints=VirtualMobileBaseJoints(),
)

################################## Left Arm ##################################
left_arm = KinematicChainDescription(
    "left", "world", "arm_t8_r0", armar_description.urdf_object, arm_type=Arms.LEFT
)

left_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "torso_joint": -0.15,
        "arm_t12_joint_r0": 0,
        "arm_t23_joint_r0": 0,
        "arm_t34_joint_r0": 1.5,
        "arm_t45_joint_r0": 0.5,
        "arm_t56_joint_r0": 2.0,
        "arm_t67_joint_r0": 1.5,
        "arm_t78_joint_r0": 0,
        "arm_t8_joint_r0": 0,
    },
)

armar_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################
left_gripper = EndEffectorDescription(
    "left_gripper", "arm_t8_r0", "left_tool_frame", armar_description.urdf_object
)
left_gripper.add_static_joint_states(
    GripperState.OPEN,
    {
        "Thumb L 1 Joint": 0.0,
        "Thumb L 2 Joint": 0.0,
        "Index L 1 Joint": 0.0,
        "Index L 2 Joint": 0.0,
        "Index L 3 Joint": 0.0,
        "Middle L 1 Joint": 0.0,
        "Middle L 2 Joint": 0.0,
        "Middle L 3 Joint": 0.0,
        "Ring L 1 Joint": 0.0,
        "Ring L 2 Joint": 0.0,
        "Ring L 3 Joint": 0.0,
        "Pinky L 1 Joint": 0.0,
        "Pinky L 2 Joint": 0.0,
        "Pinky L 3 Joint": 0.0,
    },
)
left_gripper.add_static_joint_states(
    GripperState.CLOSE,
    {
        "Thumb L 1 Joint": 1.57,
        "Thumb L 2 Joint": 1.57,
        "Index L 1 Joint": 1.57,
        "Index L 2 Joint": 1.57,
        "Index L 3 Joint": 1.57,
        "Middle L 1 Joint": 1.57,
        "Middle L 2 Joint": 1.57,
        "Middle L 3 Joint": 1.57,
        "Ring L 1 Joint": 1.57,
        "Ring L 2 Joint": 1.57,
        "Ring L 3 Joint": 1.57,
        "Pinky L 1 Joint": 1.57,
        "Pinky L 2 Joint": 1.57,
        "Pinky L 3 Joint": 1.57,
    },
)

left_gripper.end_effector_type = GripperType.FINGER
# left_gripper.opening_distance = 0.548
left_arm.end_effector = left_gripper

################################## Right Arm ##################################
right_arm = KinematicChainDescription(
    "right", "world", "arm_t8_r1", armar_description.urdf_object, arm_type=Arms.RIGHT
)

right_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "torso_joint": -0.15,
        "arm_t12_joint_r1": 0,
        "arm_t23_joint_r1": 0,
        "arm_t34_joint_r1": 1.5,
        "arm_t45_joint_r1": 2.64,
        "arm_t56_joint_r1": 2.0,
        "arm_t67_joint_r1": 1.6415,
        "arm_t78_joint_r1": 0,
        "arm_t8_joint_r1": 0,
    },
)

armar_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################
right_gripper = EndEffectorDescription(
    "right_gripper", "arm_t8_r1", "right_tool_frame", armar_description.urdf_object
)
right_gripper.add_static_joint_states(
    GripperState.OPEN,
    {
        "Thumb R 1 Joint": 0.0,
        "Thumb R 2 Joint": 0.0,
        "Index R 1 Joint": 0.0,
        "Index R 2 Joint": 0.0,
        "Index R 3 Joint": 0.0,
        "Middle R 1 Joint": 0.0,
        "Middle R 2 Joint": 0.0,
        "Middle R 3 Joint": 0.0,
        "Ring R 1 Joint": 0.0,
        "Ring R 2 Joint": 0.0,
        "Ring R 3 Joint": 0.0,
        "Pinky R 1 Joint": 0.0,
        "Pinky R 2 Joint": 0.0,
        "Pinky R 3 Joint": 0.0,
    },
)
right_gripper.add_static_joint_states(
    GripperState.CLOSE,
    {
        "Thumb R 1 Joint": 1.57,
        "Thumb R 2 Joint": 1.57,
        "Index R 1 Joint": 1.57,
        "Index R 2 Joint": 1.57,
        "Index R 3 Joint": 1.57,
        "Middle R 1 Joint": 1.57,
        "Middle R 2 Joint": 1.57,
        "Middle R 3 Joint": 1.57,
        "Ring R 1 Joint": 1.57,
        "Ring R 2 Joint": 1.57,
        "Ring R 3 Joint": 1.57,
        "Pinky R 1 Joint": 1.57,
        "Pinky R 2 Joint": 1.57,
        "Pinky R 3 Joint": 1.57,
    },
)

right_gripper.end_effector_type = GripperType.FINGER
# right_gripper.opening_distance = 0.548
right_arm.end_effector = right_gripper

################################## Torso ##################################
torso = KinematicChainDescription(
    "torso", "platform", "torso", armar_description.urdf_object
)

torso.add_static_joint_states(TorsoState.HIGH, {"torso_joint": 0.0})
torso.add_static_joint_states(TorsoState.MID, {"torso_joint": -0.185})
torso.add_static_joint_states(TorsoState.LOW, {"torso_joint": -0.365})

armar_description.add_kinematic_chain_description(torso)

################################## Camera ##################################
camera = CameraDescription(
    "Roboception",
    "Roboception",
    1.371500015258789,
    1.7365000247955322,
    0.99483,
    0.75049,
    [0, 0, 1],
)
armar_description.add_camera_description(camera)

################################## Neck ##################################
armar_description.add_kinematic_chain("neck", "lower_neck", "upper_neck")
armar_description.set_neck(yaw_joint="neck_1_yaw", pitch_joint="neck_2_pitch")


################################# Grasps ##################################
orientation = [0.707, 0.707, 0.707, 0.707]
right_gripper.update_all_grasp_orientations(orientation)
left_gripper.update_all_grasp_orientations(orientation)


################################# Additionals ##################################
# armar_description.set_costmap_offset(0)
# armar_description.set_max_reach("torso", "left_tool_frame")
# armar_description.set_palm_axis([0, 0, 1])

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(armar_description)
