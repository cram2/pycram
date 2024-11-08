from giskardpy.configs.giskard import CollisionAvoidanceConfig, RobotInterfaceConfig
from giskardpy.configs.world_config import WorldWithOmniDriveRobot
from giskardpy.data_types import Derivatives


class WorldWithPR2Config(WorldWithOmniDriveRobot):
    def __init__(self, map_name: str = 'map', localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined', drive_joint_name: str = 'brumbrum'):
        super().__init__(map_name, localization_joint_name, odom_link_name, drive_joint_name)

    def setup(self):
        super().setup()
        self.set_joint_limits(limit_map={Derivatives.velocity: 2,
                                         Derivatives.jerk: 60},
                              joint_name='head_pan_joint')
        self.set_joint_limits(limit_map={Derivatives.velocity: 4,
                                         Derivatives.jerk: 120},
                              joint_name='head_tilt_joint')



class PR2StandaloneInterface(RobotInterfaceConfig):
    drive_joint_name: str

    def __init__(self, drive_joint_name: str):
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.register_controlled_joints([
            'torso_lift_joint',
            'head_pan_joint',
            'head_tilt_joint',
            'r_shoulder_pan_joint',
            'r_shoulder_lift_joint',
            'r_upper_arm_roll_joint',
            'r_forearm_roll_joint',
            'r_elbow_flex_joint',
            'r_wrist_flex_joint',
            'r_wrist_roll_joint',
            'l_shoulder_pan_joint',
            'l_shoulder_lift_joint',
            'l_upper_arm_roll_joint',
            'l_forearm_roll_joint',
            'l_elbow_flex_joint',
            'l_wrist_flex_joint',
            'l_wrist_roll_joint',
            self.drive_joint_name,
        ])


class PR2JointTrajServerMujocoInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/joint_states')
        self.sync_odometry_topic('/pr2/base_footprint', self.drive_joint_name)
        self.add_follow_joint_trajectory_server(
            namespace='/pr2/whole_body_controller')
        self.add_follow_joint_trajectory_server(
            namespace='/pr2/l_gripper_l_finger_controller')
        self.add_follow_joint_trajectory_server(
            namespace='/pr2/r_gripper_l_finger_controller')
        self.add_base_cmd_velocity(cmd_vel_topic='/pr2/cmd_vel',
                                   track_only_velocity=True,
                                   joint_name=self.drive_joint_name)

class PR2JointTrajServerMultiverseInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'pr2',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        # self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
        #                                    tf_parent_frame=self.map_name,
                                           # tf_child_frame=self.odom_link_name)
        # self.
        self.sync_joint_state_topic('/real/pr2/joint_states')
        self.sync_odometry_topic('/odom', self.drive_joint_name)
        self.add_follow_joint_trajectory_server(
            namespace='/real/pr2/head_controller')
        self.add_follow_joint_trajectory_server(
            namespace='/real/pr2/torso_controller')
        self.add_follow_joint_trajectory_server(
            namespace='/real/pr2/left_arm_controller')
        self.add_follow_joint_trajectory_server(
            namespace='/real/pr2/right_arm_controller')
        self.add_base_cmd_velocity(cmd_vel_topic='/cmd_vel',
                                   track_only_velocity=True,
                                   joint_name=self.drive_joint_name)

class PR2VelocityMujocoInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('pr2/joint_states')
        self.sync_odometry_topic('/pr2/base_footprint', self.drive_joint_name)
        self.add_joint_velocity_controller(namespaces=[
            'pr2/torso_lift_velocity_controller',
            'pr2/r_upper_arm_roll_velocity_controller',
            'pr2/r_shoulder_pan_velocity_controller',
            'pr2/r_shoulder_lift_velocity_controller',
            'pr2/r_forearm_roll_velocity_controller',
            'pr2/r_elbow_flex_velocity_controller',
            'pr2/r_wrist_flex_velocity_controller',
            'pr2/r_wrist_roll_velocity_controller',
            'pr2/l_upper_arm_roll_velocity_controller',
            'pr2/l_shoulder_pan_velocity_controller',
            'pr2/l_shoulder_lift_velocity_controller',
            'pr2/l_forearm_roll_velocity_controller',
            'pr2/l_elbow_flex_velocity_controller',
            'pr2/l_wrist_flex_velocity_controller',
            'pr2/l_wrist_roll_velocity_controller',
            'pr2/head_pan_velocity_controller',
            'pr2/head_tilt_velocity_controller',
        ])

        self.add_base_cmd_velocity(cmd_vel_topic='/pr2/cmd_vel',
                                   joint_name=self.drive_joint_name)


class PR2VelocityIAIInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/joint_states')
        self.sync_odometry_topic('/robot_pose_ekf/odom_combined', self.drive_joint_name)
        self.add_joint_velocity_group_controller(namespace='l_arm_joint_group_velocity_controller')
        self.add_joint_velocity_group_controller(namespace='r_arm_joint_group_velocity_controller')
        self.add_joint_position_controller(namespaces=[
            'head_pan_position_controller',
            'head_tilt_position_controller',
        ])

        self.add_base_cmd_velocity(cmd_vel_topic='/base_controller/command',
                                   joint_name=self.drive_joint_name)


class PR2CollisionAvoidance(CollisionAvoidanceConfig):
    def __init__(self, drive_joint_name: str = 'brumbrum'):
        super().__init__()
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.load_self_collision_matrix('package://giskardpy/self_collision_matrices/iai/pr2.srdf')
        self.set_default_external_collision_avoidance(soft_threshold=0.1,
                                                      hard_threshold=0.0)
        for joint_name in ['r_wrist_roll_joint', 'l_wrist_roll_joint']:
            self.overwrite_external_collision_avoidance(joint_name,
                                                        number_of_repeller=4,
                                                        soft_threshold=0.05,
                                                        hard_threshold=0.0,
                                                        max_velocity=0.2)
        for joint_name in ['r_wrist_flex_joint', 'l_wrist_flex_joint']:
            self.overwrite_external_collision_avoidance(joint_name,
                                                        number_of_repeller=2,
                                                        soft_threshold=0.05,
                                                        hard_threshold=0.0,
                                                        max_velocity=0.2)
        for joint_name in ['r_elbow_flex_joint', 'l_elbow_flex_joint']:
            self.overwrite_external_collision_avoidance(joint_name,
                                                        soft_threshold=0.05,
                                                        hard_threshold=0.0)
        for joint_name in ['r_forearm_roll_joint', 'l_forearm_roll_joint']:
            self.overwrite_external_collision_avoidance(joint_name,
                                                        soft_threshold=0.025,
                                                        hard_threshold=0.0)
        self.fix_joints_for_collision_avoidance([
            'r_gripper_l_finger_joint',
            'l_gripper_l_finger_joint'
        ])
        self.overwrite_external_collision_avoidance(self.drive_joint_name,
                                                    number_of_repeller=2,
                                                    soft_threshold=0.2,
                                                    hard_threshold=0.1)


class PR2JointTrajServerIAIInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/joint_states')
        self.sync_odometry_topic('/robot_pose_ekf/odom_combined', self.drive_joint_name)
        fill_velocity_values = False
        self.add_follow_joint_trajectory_server(namespace='/l_arm_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_follow_joint_trajectory_server(namespace='/r_arm_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_follow_joint_trajectory_server(namespace='/torso_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_follow_joint_trajectory_server(namespace='/head_traj_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_base_cmd_velocity(cmd_vel_topic='/base_controller/command',
                                   track_only_velocity=True,
                                   joint_name=self.drive_joint_name)


class PR2JointTrajServerUnrealInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/joint_states')
        self.sync_odometry_topic('/base_odometry/odom', self.drive_joint_name)
        fill_velocity_values = False
        self.add_follow_joint_trajectory_server(namespace='/whole_body_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_follow_joint_trajectory_server(namespace='/head_traj_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_base_cmd_velocity(cmd_vel_topic='/base_controller/command',
                                   track_only_velocity=True,
                                   joint_name=self.drive_joint_name)
