from ..robot_description import (
    RobotDescription, RobotDescriptionManager, KinematicChainDescription,
    CameraDescription, EndEffectorDescription
)
from ..datastructures.dataclasses import VirtualMobileBaseJoints
from ..datastructures.enums import Arms, StaticJointState, TorsoState, GripperState
from ..ros import get_ros_package_path
from ..helper import get_robot_description_path

# paths
filename = get_ros_package_path('pycram') + '/resources/robots/g1/g1.urdf'
mjcf_filename = get_robot_description_path('', 'g1_comp')

# base description
g1_description = RobotDescription(
    'g1_comp',          # <robot name="g1_comp">
    'pelvis',           # base link
    'torso_link',       # torso root
    'waist_yaw_joint',  # torso yaw joint
    filename,
    VirtualMobileBaseJoints(),  # default virtual base joints for PyCRAM
    mjcf_filename,
)

# ---------------- left arm ----------------
left_arm = KinematicChainDescription(
    'left',
    'torso_link',
    'left_wrist_roll_rubber_hand',
    g1_description.urdf_object,
    arm_type=Arms.LEFT,
)
left_arm.add_static_joint_states(StaticJointState.Park, {
    'left_shoulder_pitch_joint': 0.0,
    'left_shoulder_roll_joint':  0.0,
    'left_shoulder_yaw_joint':   0.0,
    'left_elbow_joint':          0.0,
    'left_wrist_roll_joint':     0.0,
})
g1_description.add_kinematic_chain_description(left_arm)

# ---------------- right arm ----------------
right_arm = KinematicChainDescription(
    'right',
    'torso_link',
    'right_wrist_roll_rubber_hand',
    g1_description.urdf_object,
    arm_type=Arms.RIGHT,
)
right_arm.add_static_joint_states(StaticJointState.Park, {
    'right_shoulder_pitch_joint': 0.0,
    'right_shoulder_roll_joint':  0.0,
    'right_shoulder_yaw_joint':   0.0,
    'right_elbow_joint':          0.0,
    'right_wrist_roll_joint':     0.0,
})
g1_description.add_kinematic_chain_description(right_arm)

# ---------------- torso (yaw presets) ----------------
torso = KinematicChainDescription('torso', 'pelvis', 'torso_link', g1_description.urdf_object)
torso.add_static_joint_states(TorsoState.LOW,  {'waist_yaw_joint': -1.0})
torso.add_static_joint_states(TorsoState.MID,  {'waist_yaw_joint':  0.0})
torso.add_static_joint_states(TorsoState.HIGH, {'waist_yaw_joint': +1.0})
g1_description.add_kinematic_chain_description(torso)

# ---------------- head (single yaw joint) ----------------
head = KinematicChainDescription('head', 'torso_link', 'xl330_link', g1_description.urdf_object)
head.add_static_joint_states(StaticJointState.Park, {'xl330_joint': 0.0})
g1_description.add_kinematic_chain_description(head)

# ---------------- camera (on head link) ----------------
camera = CameraDescription("g1_camera_head", "xl330_link", 1.0, 1.0, 0.5, 0.5)
g1_description.add_camera_description(camera)

# ---------------- end effectors ----------------
# Using wrist links as both palm_link and tool_frame (TCP) for now.
left_gripper = EndEffectorDescription(
    "left_gripper",
    "left_wrist_roll_rubber_hand",   # palm
    "left_wrist_roll_rubber_hand",   # tool frame (temporary TCP)
    g1_description.urdf_object,
    fingers_link_names=[]
)
left_gripper.add_static_joint_states(GripperState.OPEN,  {})
left_gripper.add_static_joint_states(GripperState.CLOSE, {})
left_arm.end_effector = left_gripper

right_gripper = EndEffectorDescription(
    "right_gripper",
    "right_wrist_roll_rubber_hand",  # palm
    "right_wrist_roll_rubber_hand",  # tool frame (temporary TCP)
    g1_description.urdf_object,
    fingers_link_names=[]
)
right_gripper.add_static_joint_states(GripperState.OPEN,  {})
right_gripper.add_static_joint_states(GripperState.CLOSE, {})
right_arm.end_effector = right_gripper

# Optional grasp semantics (only if available in your build)
if hasattr(right_gripper, "update_all_grasp_orientations"):
    right_gripper.update_all_grasp_orientations([0, 0, 0, 1])
if hasattr(right_gripper, "set_approach_axis"):
    # Assume +X of wrist points forward
    right_gripper.set_approach_axis([1, 0, 0])

# register
RobotDescriptionManager().register_description(g1_description)
