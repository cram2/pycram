from ..ros import get_ros_package_path
from ..robot_description import (
    RobotDescription,
    KinematicChainDescription,
    EndEffectorDescription,
    RobotDescriptionManager,
)
from ..datastructures.enums import (
    Arms,
    Grasp,
    GripperState,
    GripperType,
    StaticJointState,
)
from ..units import meter

filename = (
    get_ros_package_path("pycram") + "/resources/robots/" + "ur5_robotiq" + ".urdf"
)

ur5_description = RobotDescription(
    "ur5_robotiq", "world", "base_link", "ee_link", filename
)

################################## Arm ##################################
arm = KinematicChainDescription(
    "manipulator",
    "base_link",
    "wrist_3_link",
    ur5_description.urdf_object,
    arm_type=Arms.RIGHT,
)

arm.add_static_joint_states(
    StaticJointState.Park,
    {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": 0.0,
        "elbow_joint": 0.0,
        "wrist_1_joint": 0.0,
        "wrist_2_joint": 0.0,
        "wrist_3_joint": 0.0,
    },
)

ur5_description.add_kinematic_chain_description(arm)

################################## Gripper ##################################
gripper = EndEffectorDescription(
    "gripper",
    "robotiq_85_base_link",
    "robotiq_85_right_finger_link",
    ur5_description.urdf_object,
)

gripper.add_static_joint_states(
    GripperState.OPEN,
    {
        "robotiq_85_left_finger_joint": 0.0,
        "robotiq_85_right_finger_joint": 0.0,
        "robotiq_85_left_inner_knuckle_joint": 0.0,
        "robotiq_85_right_inner_knuckle_joint": 0.0,
        "robotiq_85_left_finger_tip_joint": 0.0,
        "robotiq_85_right_finger_tip_joint": 0.0,
    },
)
gripper.add_static_joint_states(
    GripperState.CLOSE,
    {
        "robotiq_85_left_finger_joint": 1,
        "robotiq_85_right_finger_joint": 1,
        "robotiq_85_left_inner_knuckle_joint": 1.0,
        "robotiq_85_right_inner_knuckle_joint": 1.0,
        "robotiq_85_left_finger_tip_joint": 1.0,
        "robotiq_85_right_finger_tip_joint": 1.0,
    },
)
gripper.end_effector_type = GripperType.PARALLEL
gripper.opening_distance = 0.085 * meter
arm.end_effector = gripper

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(ur5_description)
