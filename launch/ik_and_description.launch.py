import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

pkg_share = FindPackageShare('trac_ik_examples').find('trac_ik_examples')
urdf_file = os.path.join(pkg_share, 'launch', 'pr2.urdf')
with open(urdf_file, 'r') as infp:
    robot_desc = infp.read()


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trac_ik_service',
            executable='start_ik_service',
            name='ik_node',
            output='screen',
            parameters=[{
                'robot_description': robot_desc
            }]
        ),
    ])


# def generate_launch_description():
#     nodes = []
#     #robot = LaunchConfiguration("robot")
#     robot_launch_arg = DeclareLaunchArgument(
#         'robot',
#         default_value=TextSubstitution(text='default'),
#     )
#
#     return LaunchDescription([
#         Node(
#             package='kdl_ik_service',
#             executable='start_ros_server',
#             name='ik_node',
#             output='screen',
#         ),
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(
#                     get_package_share_directory('pr2_arm_kinematics'),
#                     'launch/pr2_ik_larm_node.launch'))
#         ),
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(
#                     get_package_share_directory('pr2_arm_kinematics'),
#                     'launch/pr2_ik_rarm_node.launch'))
#         ),
#         Node(
#             condition=IfCondition(
#                 PythonExpression([
#                     robot_launch_arg,
#                     ' == ',
#                     'pr2'])
#             ),
#             package='iai_pr2_description',
#             executable='upload_pr2.launch',
#             name='pr2_description',
#             output='screen'
#         )
#     ])