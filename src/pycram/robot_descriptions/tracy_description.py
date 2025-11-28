from ..robot_description import (
    RobotDescription,
    KinematicChainDescription,
    EndEffectorDescription,
    CameraDescription,
)
from ..datastructures.enums import Arms, GripperState, GripperType, StaticJointState
from ..ros import get_ros_package_path
from ..robot_description import RobotDescriptionManager
from ..units import meter

filename = get_ros_package_path("pycram") + "/resources/robots/tracy.urdf"
tracy_description = RobotDescription(
    "tracy", "world", "table", "table_joint", filename, ignore_joints=[]
)

# left and right arm descriptions
left_arm = KinematicChainDescription(
    "left_arm",
    "table",
    "left_wrist_3_link",
    tracy_description.urdf_object,
    arm_type=Arms.LEFT,
)
right_arm = KinematicChainDescription(
    "right_arm",
    "table",
    "right_wrist_3_link",
    tracy_description.urdf_object,
    arm_type=Arms.RIGHT,
)

left_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "left_shoulder_pan_joint": 3.0,
        "left_shoulder_lift_joint": -1,
        "left_elbow_joint": 1.2,
        "left_wrist_1_joint": -0.5,
        "left_wrist_2_joint": 1.57,
        "left_wrist_3_joint": 0.0,
    },
)

right_arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "right_shoulder_pan_joint": 3,
        "right_shoulder_lift_joint": -2.1,
        "right_elbow_joint": -1.57,
        "right_wrist_1_joint": 0.5,
        "right_wrist_2_joint": 1.57,
        "right_wrist_3_joint": 0.0,
    },
)

# left and right gripper descriptions
left_gripper = EndEffectorDescription(
    "left_gripper",
    "left_robotiq_85_base_link",
    "l_gripper_tool_frame",
    tracy_description.urdf_object,
    fingers_link_names=[
        "left_robotiq_85_left_finger_tip_link",
        "left_robotiq_85_right_finger_tip_link",
    ],
)
right_gripper = EndEffectorDescription(
    "right_gripper",
    "right_robotiq_85_base_link",
    "r_gripper_tool_frame",
    tracy_description.urdf_object,
    fingers_link_names=[
        "right_robotiq_85_left_finger_tip_link",
        "right_robotiq_85_right_finger_tip_link",
    ],
)
# Configure gripper states
left_gripper.add_static_joint_states(
    GripperState.OPEN,
    {
        "left_robotiq_85_left_knuckle_joint": 0,
        "left_robotiq_85_right_knuckle_joint": 0,
        "left_robotiq_85_left_inner_knuckle_joint": 0,
        "left_robotiq_85_right_inner_knuckle_joint": 0,
        "left_robotiq_85_left_finger_tip_joint": 0,
        "left_robotiq_85_right_finger_tip_joint": 0,
    },
)
left_gripper.add_static_joint_states(
    GripperState.CLOSE,
    {
        "left_robotiq_85_left_knuckle_joint": 0.8,
        "left_robotiq_85_right_knuckle_joint": -0.8,
        "left_robotiq_85_left_inner_knuckle_joint": -0.8,
        "left_robotiq_85_right_inner_knuckle_joint": 0.8,
        "left_robotiq_85_left_finger_tip_joint": -0.8,
        "left_robotiq_85_right_finger_tip_joint": 0.8,
    },
)
left_gripper.end_effector_type = GripperType.PARALLEL
left_gripper.opening_distance = 0.085 * meter


right_gripper.add_static_joint_states(
    GripperState.OPEN,
    {
        "right_robotiq_85_left_knuckle_joint": 0,
        "right_robotiq_85_right_knuckle_joint": 0,
        "right_robotiq_85_left_inner_knuckle_joint": 0,
        "right_robotiq_85_right_inner_knuckle_joint": 0,
        "right_robotiq_85_left_finger_tip_joint": 0,
        "right_robotiq_85_right_finger_tip_joint": 0,
    },
)
right_gripper.add_static_joint_states(
    GripperState.CLOSE,
    {
        "right_robotiq_85_left_knuckle_joint": 0.8,
        "right_robotiq_85_right_knuckle_joint": -0.8,
        "right_robotiq_85_left_inner_knuckle_joint": -0.8,
        "right_robotiq_85_right_inner_knuckle_joint": 0.8,
        "right_robotiq_85_left_finger_tip_joint": -0.8,
        "right_robotiq_85_right_finger_tip_joint": 0.8,
    },
)

right_gripper.end_effector_type = GripperType.PARALLEL
right_gripper.opening_distance = 0.085 * meter

right_gripper.update_all_grasp_orientations([0, 0, 0, 1])
left_gripper.update_all_grasp_orientations([0, 0, 0, 1])
right_gripper.set_approach_axis([1, 0, 0])
left_gripper.set_approach_axis([1, 0, 0])
# Attach grippers to arms
left_arm.end_effector = left_gripper
right_arm.end_effector = right_gripper

# Add kinematic chains to robot description
tracy_description.add_kinematic_chain_description(left_arm)
tracy_description.add_kinematic_chain_description(right_arm)


# Camera description
camera = CameraDescription(
    "head_camera",
    "camera_link",
    0.8,
    1.7,
    1.047,
    0.785,
)
tracy_description.add_camera_description(camera)

# Register with RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(tracy_description)
