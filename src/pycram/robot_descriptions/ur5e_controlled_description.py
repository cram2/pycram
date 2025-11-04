from ..datastructures.dataclasses import ManipulatorData
from ..helper import get_robot_urdf_and_mjcf_file_paths, find_multiverse_resources_path
from ..robot_description import RobotDescriptionManager, create_manipulator_description
from ..logging import  logwarn, loginfo

data = ManipulatorData(
    name="ur5e",
    relative_dir="universal_robot",
    base_link="base_link",

    arm_end_link="wrist_3_link",
    joint_names=['shoulder_pan_joint',
                 'shoulder_lift_joint',
                 'elbow_joint',
                 'wrist_1_joint',
                 'wrist_2_joint',
                 'wrist_3_joint'],
    home_joint_values=[3.14, -1.56, 1.58, -1.57, -1.57, 0.0],

    gripper_name="gripper-2F-85",
    gripper_relative_dir="robotiq",
    gripper_tool_frame="right_pad",

    gripper_joint_names=['right_driver_joint',
                         'right_coupler_joint',
                         'right_spring_link_joint',
                         'right_follower_joint',
                         'left_driver_joint',
                         'left_coupler_joint',
                         'left_spring_link_joint',
                         'left_follower_joint'],
    closed_joint_values=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    open_joint_values=[0.798, 0.00366, 0.796, -0.793, 0.798, 0.00366, 0.796, -0.793],
    opening_distance=None,  # TODO: Check this value

    gripper_cmd_topic="/gripper_command",
    gripper_open_cmd_value=0.0,
    gripper_close_cmd_value=255.0)

multiverse_resources = find_multiverse_resources_path()

urdf_filename, mjcf_filename = None, None
if multiverse_resources is not None:
    urdf_filename, mjcf_filename = get_robot_urdf_and_mjcf_file_paths(data.name, data.relative_dir,
                                                                      multiverse_resources=multiverse_resources)

if mjcf_filename is None or urdf_filename is None:
    loginfo(f"Could not initialize {data.name} description as Multiverse resources path not found.")
else:
    robot_description = create_manipulator_description(data, urdf_filename, mjcf_filename)

    # Add to RobotDescriptionManager
    rdm = RobotDescriptionManager()
    rdm.register_description(robot_description)
