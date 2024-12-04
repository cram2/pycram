import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from vtkmodules.numpy_interface.algorithms import condition


def generate_launch_description():
    nodes = []
    #robot = LaunchConfiguration("robot")
    robot_launch_arg = DeclareLaunchArgument(
        'robot',
        default_value=TextSubstitution(text='default'),
    )

    return LaunchDescription([
        Node(
            package='kdl_ik_service',
            executable='start_ros_server',
            name='ik_node',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('pr2_arm_kinematics'),
                    'launch/pr2_ik_larm_node.launch'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('pr2_arm_kinematics'),
                    'launch/pr2_ik_rarm_node.launch'))
        ),
        Node(
            condition=IfCondition(
                PythonExpression([
                    robot_launch_arg,
                    ' == ',
                    'pr2'])
            ),
            package='iai_pr2_description',
            executable='upload_pr2.launch',
            name='pr2_description',
            output='screen'
        )
    ])