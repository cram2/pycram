from ..robot_description import (
    RobotDescription,
    KinematicChainDescription,
    EndEffectorDescription,
    CameraDescription,
    RobotDescriptionManager,
)
from ..ros import get_ros_package_path
from ..datastructures.enums import Arms, Grasp, GripperState, GripperType, TorsoState
from ..datastructures.dataclasses import VirtualMobileBaseJoints
from ..units import meter

filename = get_ros_package_path("pycram") + "/resources/robots/" + "kevin" + ".urdf"
kevin_description = RobotDescription(
    "kevin",
    "robot_base_footprint",
    "robot_base_link",
    "robot_arm_column_joint",
    filename,
    virtual_mobile_base_joints=VirtualMobileBaseJoints(),
)

################################## Arm ##################################
left_arm_description = KinematicChainDescription(
    "left",
    "robot_arm_base_link",
    "robot_arm_wrist_link",
    kevin_description.urdf_object,
    arm_type=Arms.LEFT,
)

left_arm_description.add_static_joint_states(
    "park",
    {
        "robot_arm_column_joint": 0.63,
        "robot_arm_inner_joint": 0.03,
        "robot_arm_outer_joint": 4.70,
        "robot_arm_wrist_joint": -1.63,
    },
)

kevin_description.add_kinematic_chain_description(left_arm_description)

################################## Gripper ##################################
only_gripper_description = EndEffectorDescription(
    "gripper",
    "robot_arm_wrist_link",
    "robot_arm_tool_link",
    kevin_description.urdf_object,
)
only_gripper_description.add_static_joint_states(
    GripperState.OPEN,
    {"robot_arm_gripper_joint": 0.066, "robot_arm_gripper_mirror_joint": 0.066},
)
only_gripper_description.add_static_joint_states(
    GripperState.CLOSE,
    {"robot_arm_gripper_joint": 0.0, "robot_arm_gripper_mirror_joint": 0.0},
)
only_gripper_description.end_effector_type = GripperType.PARALLEL
only_gripper_description.opening_distance = 0.1 * meter

left_arm_description.end_effector = only_gripper_description

################################## Cameras ##################################
min_heigh = 1.2
max_height = 1.21
horizontal_angle = 0.99483  # copied from pr2
vertical_angle = 0.75049  # copied from pr2
front_facing_axis = [1, 0, 0]

top_cam = CameraDescription(
    "top_cam",
    "robot_top_3d_laser_link",
    min_heigh,
    max_height,
    horizontal_angle,
    vertical_angle,
    front_facing_axis,
)

kevin_description.add_camera_description(top_cam)

################################## Neck ##################################
# Everyone elese has a neck? Kevin doesn't really
# kevin_description.add_kinematic_chain("neck", "robot_front_rgbd_camera_base_link", "robot_front_rgbd_camera_link")
################################## Torso ##################################
torso = KinematicChainDescription(
    "torso", "robot_base_link", "robot_arm_column_link", kevin_description.urdf_object
)

torso.add_static_joint_states(TorsoState.HIGH, {"robot_arm_column_joint": 0.69})

torso.add_static_joint_states(TorsoState.MID, {"robot_arm_column_joint": 0.3})

torso.add_static_joint_states(TorsoState.LOW, {"robot_arm_column_joint": 0})

kevin_description.add_kinematic_chain_description(torso)
################################# Grasps ##################################
only_gripper_description.update_all_grasp_orientations([0, 0, 0, 1])

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(kevin_description)
