from semantic_world.prefixed_name import PrefixedName
from semantic_world.robots import PR2

from ..datastructures.enums import StaticJointState, Arms, GripperState as GripperStateEnum, TorsoState
from ..joint_state import JointState, ArmState, GripperState, JointStateManager

right_park = ArmState(
    name=PrefixedName("pr2", "right_park"),
    joint_names=["r_shoulder_pan_joint", "r_shoulder_lift_joint",
                 "r_upper_arm_roll_joint", "r_elbow_flex_joint",
                 "r_forearm_roll_joint", "r_wrist_flex_joint",
                 "r_wrist_roll_joint"],
    joint_positions=[-1.712, -0.256, -1.463, -2.12, 1.766, -0.07, 0.051],
    state_type=StaticJointState.Park,
    arm=Arms.RIGHT
)

left_park = ArmState(
    name=PrefixedName("pr2", "left_park"),
    joint_names=["l_shoulder_pan_joint", "l_shoulder_lift_joint",
                 "l_upper_arm_roll_joint", "l_elbow_flex_joint",
                 "l_forearm_roll_joint", "l_wrist_flex_joint",
                 "l_wrist_roll_joint"],
    joint_positions=[1.712, -0.264, 1.38, -2.12, 16.996 + 3.14159, -0.073, 0.0],
    state_type=StaticJointState.Park,
    arm=Arms.LEFT
)

both_park = ArmState(
    name=PrefixedName("pr2", "both_park"),
    joint_names=["l_shoulder_pan_joint", "l_shoulder_lift_joint",
                 "l_upper_arm_roll_joint", "l_elbow_flex_joint",
                 "l_forearm_roll_joint", "l_wrist_flex_joint",
                 "l_wrist_roll_joint",
                 "r_shoulder_pan_joint", "r_shoulder_lift_joint",
                 "r_upper_arm_roll_joint", "r_elbow_flex_joint",
                 "r_forearm_roll_joint", "r_wrist_flex_joint",
                 "r_wrist_roll_joint"],
    joint_positions=[1.712, -0.264, 1.38, -2.12, 16.996 + 3.14159, -0.073, 0.0,
                     -1.712, -0.256, -1.463, -2.12, 1.766, -0.07, 0.051],
    state_type=StaticJointState.Park,
    arm=Arms.BOTH
)

left_gripper_open = GripperState(
    name=PrefixedName("pr2", "left_gripper_open"),
    joint_names=["l_gripper_l_finger_joint", "l_gripper_r_finger_joint"],
    joint_positions=[0.548, 0.548],
    state_type=GripperStateEnum.OPEN,
    gripper=Arms.LEFT
)

left_gripper_close = GripperState(
    name=PrefixedName("pr2", "left_gripper_close"),
    joint_names=["l_gripper_l_finger_joint", "l_gripper_r_finger_joint"],
    joint_positions=[0.0, 0.0],
    state_type=GripperStateEnum.CLOSE,
    gripper=Arms.LEFT
)

right_gripper_open = GripperState(
    name=PrefixedName("pr2", "right_gripper_open"),
    joint_names=["r_gripper_l_finger_joint", "r_gripper_r_finger_joint"],
    joint_positions=[0.548, 0.548],
    state_type=GripperStateEnum.OPEN,
    gripper=Arms.RIGHT
)

right_gripper_close = GripperState(
    name=PrefixedName("pr2", "right_gripper_close"),
    joint_names=["r_gripper_l_finger_joint", "r_gripper_r_finger_joint"],
    joint_positions=[0.0, 0.0],
    state_type=GripperStateEnum.CLOSE,
    gripper=Arms.RIGHT
)

torso_low = JointState(
    name=PrefixedName("pr2", "torso_low"),
    joint_names=["torso_lift_joint"],
    joint_positions=[0.0],
    state_type=TorsoState.LOW)

torso_mid = JointState(
    name=PrefixedName("pr2", "torso_mid"),
    joint_names=["torso_lift_joint"],
    joint_positions=[0.15],
    state_type=TorsoState.MID)

torso_high = JointState(
    name=PrefixedName("pr2", "torso_high"),
    joint_names=["torso_lift_joint"],
    joint_positions=[0.3],
    state_type=TorsoState.HIGH)

JointStateManager().add_joint_states(PR2, [right_park, left_park, both_park, right_gripper_open, right_gripper_close,
                                           left_gripper_open, left_gripper_close, torso_low, torso_mid, torso_high])
