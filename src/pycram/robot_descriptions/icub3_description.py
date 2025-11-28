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

filename = get_ros_package_path("pycram") + "/resources/robots/" + "iCub3" + ".urdf"

icub_description = RobotDescription(
    "iCub3", "base_footprint", "torso_1", "torso_pitch", filename
)

################################## Left Arm ##################################
left_arm = KinematicChainDescription(
    "left", "root_link", "l_hand", icub_description.urdf_object, arm_type=Arms.LEFT
)

left_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "torso_roll": 0,
        "torso_pitch": 0,
        "torso_yaw": 0,
        "l_shoulder_pitch": 0,
        "l_shoulder_roll": 0,
        "l_shoulder_yaw": 0,
        "l_elbow": 0,
        "l_wrist_prosup": 0,
        "l_wrist_pitch": 0,
        "l_wrist_yaw": 0,
    },
)

icub_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################
left_gripper = EndEffectorDescription(
    "left_gripper", "l_hand", "l_gripper_tool_frame", icub_description.urdf_object
)
left_gripper.add_static_joint_states(
    GripperState.OPEN,
    {
        "l_hand_thumb_0_joint": 0.0,
        "l_hand_thumb_1_joint": 0.0,
        "l_hand_thumb_2_joint": 0.0,
        "l_hand_thumb_3_joint": 0.0,
        "l_hand_index_0_joint": 0.0,
        "l_hand_index_1_joint": 0.0,
        "l_hand_index_2_joint": 0.0,
        "l_hand_index_3_joint": 0.0,
        "l_hand_middle_0_joint": 0.0,
        "l_hand_middle_1_joint": 0.0,
        "l_hand_middle_2_joint": 0.0,
        "l_hand_middle_3_joint": 0.0,
        "l_hand_ring_0_joint": 0.0,
        "l_hand_ring_1_joint": 0.0,
        "l_hand_ring_2_joint": 0.0,
        "l_hand_ring_3_joint": 0.0,
        "l_hand_little_0_joint": 0.0,
        "l_hand_little_1_joint": 0.0,
        "l_hand_little_2_joint": 0.0,
        "l_hand_little_3_joint": 0.0,
    },
)
left_gripper.add_static_joint_states(
    GripperState.CLOSE,
    {
        "l_hand_thumb_0_joint": 1.5707963267948966,
        "l_hand_thumb_1_joint": 1.5707963267948966,
        "l_hand_thumb_2_joint": 1.5707963267948966,
        "l_hand_thumb_3_joint": 1.5707963267948966,
        "l_hand_index_0_joint": -0.3490658503988659,
        "l_hand_index_1_joint": 1.5707963267948966,
        "l_hand_index_2_joint": 1.5707963267948966,
        "l_hand_index_3_joint": 1.5707963267948966,
        "l_hand_middle_0_joint": 0.3490658503988659,
        "l_hand_middle_1_joint": 1.5707963267948966,
        "l_hand_middle_2_joint": 1.5707963267948966,
        "l_hand_middle_3_joint": 1.5707963267948966,
        "l_hand_ring_0_joint": 0.3490658503988659,
        "l_hand_ring_1_joint": 1.5707963267948966,
        "l_hand_ring_2_joint": 1.5707963267948966,
        "l_hand_ring_3_joint": 1.5707963267948966,
        "l_hand_little_0_joint": 0.3490658503988659,
        "l_hand_little_1_joint": 1.5707963267948966,
        "l_hand_little_2_joint": 1.5707963267948966,
        "l_hand_little_3_joint": 1.5707963267948966,
    },
)

left_gripper.end_effector_type = GripperType.FINGER
# left_gripper.opening_distance = 0.548
left_arm.end_effector = left_gripper

################################## Right Arm ##################################
right_arm = KinematicChainDescription(
    "right", "root_link", "r_hand", icub_description.urdf_object, arm_type=Arms.RIGHT
)

right_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "torso_roll": 0,
        "torso_pitch": 0,
        "torso_yaw": 0,
        "r_shoulder_pitch": 0,
        "r_shoulder_roll": 0,
        "r_shoulder_yaw": 0,
        "r_elbow": 0,
        "r_wrist_prosup": 0,
        "r_wrist_pitch": 0,
        "r_wrist_yaw": 0,
    },
)

icub_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################
right_gripper = EndEffectorDescription(
    "right_gripper", "r_hand", "r_gripper_tool_frame", icub_description.urdf_object
)
right_gripper.add_static_joint_states(
    GripperState.OPEN,
    {
        "r_hand_thumb_0_joint": 0.0,
        "r_hand_thumb_1_joint": 0.0,
        "r_hand_thumb_2_joint": 0.0,
        "r_hand_thumb_3_joint": 0.0,
        "r_hand_index_0_joint": 0.0,
        "r_hand_index_1_joint": 0.0,
        "r_hand_index_2_joint": 0.0,
        "r_hand_index_3_joint": 0.0,
        "r_hand_middle_0_joint": 0.0,
        "r_hand_middle_1_joint": 0.0,
        "r_hand_middle_2_joint": 0.0,
        "r_hand_middle_3_joint": 0.0,
        "r_hand_ring_0_joint": 0.0,
        "r_hand_ring_1_joint": 0.0,
        "r_hand_ring_2_joint": 0.0,
        "r_hand_ring_3_joint": 0.0,
        "r_hand_little_0_joint": 0.0,
        "r_hand_little_1_joint": 0.0,
        "r_hand_little_2_joint": 0.0,
        "r_hand_little_3_joint": 0.0,
    },
)
right_gripper.add_static_joint_states(
    GripperState.CLOSE,
    {
        "r_hand_thumb_0_joint": 1.5707963267948966,
        "r_hand_thumb_1_joint": 1.5707963267948966,
        "r_hand_thumb_2_joint": 1.5707963267948966,
        "r_hand_thumb_3_joint": 1.5707963267948966,
        "r_hand_index_0_joint": -0.3490658503988659,
        "r_hand_index_1_joint": 1.5707963267948966,
        "r_hand_index_2_joint": 1.5707963267948966,
        "r_hand_index_3_joint": 1.5707963267948966,
        "r_hand_middle_0_joint": 0.3490658503988659,
        "r_hand_middle_1_joint": 1.5707963267948966,
        "r_hand_middle_2_joint": 1.5707963267948966,
        "r_hand_middle_3_joint": 1.5707963267948966,
        "r_hand_ring_0_joint": 0.3490658503988659,
        "r_hand_ring_1_joint": 1.5707963267948966,
        "r_hand_ring_2_joint": 1.5707963267948966,
        "r_hand_ring_3_joint": 1.5707963267948966,
        "r_hand_little_0_joint": 0.3490658503988659,
        "r_hand_little_1_joint": 1.5707963267948966,
        "r_hand_little_2_joint": 1.5707963267948966,
        "r_hand_little_3_joint": 1.5707963267948966,
    },
)

right_gripper.end_effector_type = GripperType.FINGER
# right_gripper.opening_distance = 0.548
right_arm.end_effector = right_gripper

################################## Torso ##################################
# redo to use knees instead of torso
torso = KinematicChainDescription(
    "torso", "root_link", "chest", icub_description.urdf_object
)

torso.add_static_joint_states(TorsoState.HIGH, {"torso_roll": 0})

torso.add_static_joint_states(TorsoState.MID, {"torso_roll": 0})

torso.add_static_joint_states(TorsoState.LOW, {"torso_roll": 0})

icub_description.add_kinematic_chain_description(torso)

################################## Camera ##################################
# real camera unknown at the moment of writing (also missing in urdf), so using dummy camera for now
camera = CameraDescription(
    "eye_camera", "head", 1.27, 1.85, 0.99483, 0.75049, [1, 0, 0]
)
icub_description.add_camera_description(camera)

################################## Neck ##################################
icub_description.add_kinematic_chain("neck", "chest", "head")
icub_description.set_neck(
    yaw_joint="neck_yaw", pitch_joint="neck_pitch", roll_joint="neck_roll"
)

################################# Grasps ##################################
left_orientation = [0.5, 0.5, 0.5, 0.5]
left_gripper.update_all_grasp_orientations(left_orientation)

right_orientation = [0, 0, -0.707, 0.707]
right_gripper.update_all_grasp_orientations(right_orientation)

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(icub_description)
