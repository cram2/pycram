from copy import deepcopy
from rospy import logerr, logwarn


class ChainDescription:
    """
    This class saves a kinematic chain by saving the links and the joints.
    Moreover, this class offers to set a base_link and specify static joint states
    (e. g. the "park" state of the right arm).

        chain of links: link1 <-> joint1 <-> link2 <-> joint2 <-> ... <-> link6
    """

    def __init__(self, name, joints, links,
                 base_link=None, static_joint_states=None):
        """
        :param name: str
        :param joints: list[str]
        :param links: list[str]
        :param static_joint_states: dict[str: list[float]]
        """
        self.name = name
        self.base_link = base_link
        self.joints = joints
        self.links = links
        self.static_joint_states = static_joint_states if static_joint_states else {}

    def add_static_joint_chain(self, configuration, static_joint_states):
        """
        :param configuration: str
        :param static_joint_states: list[float]
        """
        if len(static_joint_states) == len(self.joints):
            configurations = list(self.static_joint_states.keys())
            if configuration not in configurations:
                self.static_joint_states = self.static_joint_states
                self.static_joint_states[configuration] = static_joint_states
            else:
                logwarn("(robot_description) The chain %s already has static joint values "
                        "for the config %s.", self.name, configuration)
        else:
            logerr("(robot_description) The number of the joint values does not equal the amount of the joints.")

    def get_static_joint_chain(self, configuration):
        """
        :return: dict[str: float]
        """
        try:
            joint_values = self.static_joint_states[configuration]
        except KeyError:
            logerr("(robot_description) Configuration %s is unknown for the chain %s.", configuration, self.name)
            return None
        return dict(zip(self.joints, joint_values))


class GripperDescription(ChainDescription):

    def __init__(self, name, gripper_links=None, gripper_joints=None,
                 static_gripper_joint_states=None,
                 gripper_meter_to_jnt_multiplier=1.0, gripper_minimal_position=0.0,
                 gripper_convergence_delta=0.001):
        """
        :param gripper_links: list[str]
        :param gripper_joints: list[str]
        :param static_gripper_joint_states: dict[str: list[float]]
        """
        super().__init__(name, gripper_joints, gripper_links, base_link=None,
                         static_joint_states=static_gripper_joint_states)
        # static params
        self.gripper_meter_to_jnt_multiplier = gripper_meter_to_jnt_multiplier
        self.gripper_minimal_position = gripper_minimal_position
        self.gripper_convergence_delta = gripper_convergence_delta


class InteractionDescription(ChainDescription):
    """
    This class allows to put on the end of an chain another link, which is saved
    as an end effector. Therefore, chains can be defined which specify the interaction
    frame for the robot. An example could be a storage place for grasped objects on the robot.

        chain of links: chain_description <-> eef_link
    """

    def __init__(self, chain_description: ChainDescription, eef_link: str):
        tmp = deepcopy(chain_description)
        super().__init__(tmp.name, tmp.joints, tmp.links, tmp.static_joint_states)
        self.chain_description = tmp
        self.eef_link = eef_link


class ManipulatorDescription(InteractionDescription):
    """
    This class allows with the given interaction description to include a gripper
    description which is placed between the last link of the interaction description
    and the rest of it.
    Independently from that a tool frame can be saved, which allows to use objects
    to manipulate the environment.

        chain of links: interaction_description <-> (gripper_description) <-> (tool_frame)/eef_link
    """

    def __init__(self, interaction_description: InteractionDescription,
                 tool_frame: str = None, gripper_description: GripperDescription = None):
        tmp = deepcopy(interaction_description)
        super().__init__(tmp.chain_description, tmp.eef_link)
        self.interaction_description = tmp
        self.tool_frame = tool_frame
        self.gripper = gripper_description


class CameraDescription:

    def __init__(self, frame: str,
                 minimal_height=0.0, maximal_height=0.0,
                 vertical_angle=0.0, horizontal_angle=0.0,
                 other_params=None):
        """
        :param other_params: dict[str: float]
        """
        self.frame = frame
        self.min_height = minimal_height
        self.max_height = maximal_height
        self.vertical_angle = vertical_angle
        self.horizontal_angle = horizontal_angle
        # static, but flexible params
        self.params = other_params if other_params else {}


class RobotDescription:

    def __init__(self, odom_frame, base_frame, base_link, torso_link, torso_joint):
        self.chains = {}  # dict{str: ChainDescription}
        self.cameras = {}  # dict{str: CameraDescription}
        self.static_transforms = []  # list[tf]
        self.static_poses = []  # list[pose]
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.base_link = base_link
        self.torso_link = torso_link
        self.torso_joint = torso_joint

    def get_chain_description(self, chain_name, description_type=ChainDescription, is_same_description_type=True):
        """
        :return: subclass of ChainDescription or None
        """
        if issubclass(description_type, ChainDescription):
            try:
                chain_description = self.chains[chain_name]
            except KeyError:
                logerr("(robot_description) Name %s is unknown.", chain_name)
                return None
            if not is_same_description_type:
                return chain_description
            else:
                if type(chain_description) is description_type:
                    return chain_description
                else:
                    logerr("(robot_description) The chain %s is not of type %s, but of type %s.",
                           chain_name, description_type, type(chain_description))
        else:
            logwarn("(robot_description) Only subclasses of ChainDescription are allowed.")

    def get_tool_frame(self, manipulator_name):
        manipulator_description = self.get_chain_description(manipulator_name, description_type=ManipulatorDescription)
        if manipulator_description:
            return manipulator_description.tool_frame
        else:
            exit(0)

    def get_static_joint_chain(self, chain_name: str, configuration: str):
        """
        :return: dict[str: float]
        """
        chain_description = self.get_chain_description(chain_name, is_same_description_type=False)
        if chain_description:
            return chain_description.get_static_joint_chain(configuration)
        else:
            exit(0)

    def get_static_tf(self, base_link: str, target_link: str):
        pass

    def get_static_pose(self, frame: str):
        pass

    def add_chain(self, name: str, chain_description: ChainDescription):
        pass

    def add_camera(self, name: str, camera_description: CameraDescription):
        pass

    def get_camera_frame(self, camera_name):
        try:
            camera_description = self.cameras[camera_name]
        except KeyError:
            logerr("(robot_description) Camera name %s is unknown.", camera_name)
            return None
        return camera_description.frame

    def add_static_joint_chain(self, chain_name: str, configuration: str, static_joint_states):
        """
        :param static_joint_states: list[float]
        """
        self.chains[chain_name].add_static_joint_chain(configuration, static_joint_states)

    def add_static_gripper_chain(self, manipulator_name: str, configuration: str,
                                 static_joint_states):
        """
        :param static_joint_states: dict[str: list[float]]
        """
        manipulator_description = self.get_chain_description(manipulator_name, ManipulatorDescription)
        if manipulator_description and manipulator_description.gripper:
            manipulator_description.gripper.add_static_joint_chain(configuration, static_joint_states)

    def add_static_gripper_chains(self, manipulator_name: str, static_joint_states):
        """
        :param static_joint_states: dict[str: list[float]]
        """
        for configuration, joint_states in static_joint_states.items():
            self.add_static_gripper_chain(manipulator_name, configuration, joint_states)

    def get_static_gripper_chain(self, manipulator_name: str, configuration: str):
        manipulator_description = self.get_chain_description(manipulator_name, ManipulatorDescription)
        if manipulator_description and manipulator_description.gripper:
            return manipulator_description.gripper.get_static_joint_chain(configuration)


class PR2Description(RobotDescription):

    def __init__(self):
        super().__init__("odom_combined", "base_footprint", "base_link",
                         "torso_lift_link", "torso_lift_joint")
        # Camera
        minimal_height = 1.27
        maximal_height = 1.60
        horizontal_angle = 0.99483
        vertical_angle = 0.75049
        kinect = CameraDescription("head_mount_kinect_rgb_optical_frame",
                                   horizontal_angle=horizontal_angle, vertical_angle=vertical_angle,
                                   minimal_height=minimal_height, maximal_height=maximal_height)
        openni = CameraDescription("openni_rgb_optical_frame",
                                   horizontal_angle=horizontal_angle, vertical_angle=vertical_angle,
                                   minimal_height=minimal_height, maximal_height=maximal_height)
        stereo = CameraDescription("narrow_stereo_optical_frame",
                                   horizontal_angle=horizontal_angle, vertical_angle=vertical_angle,
                                   minimal_height=minimal_height, maximal_height=maximal_height)
        wide_stereo_optical_frame = CameraDescription("wide_stereo_optical_frame",
                                                      horizontal_angle=horizontal_angle, vertical_angle=vertical_angle,
                                                      minimal_height=minimal_height, maximal_height=maximal_height)
        self.cameras["kinect"] = kinect
        self.cameras["openni"] = openni
        self.cameras["stereo"] = stereo
        self.cameras["optical_frame"] = wide_stereo_optical_frame
        # Chains
        neck_static_joints = {}
        neck_static_joints["forward"] = [0.0, 0.0]
        neck = ChainDescription("neck", ["head_pan_joint", "head_tilt_joint"],
                                ["head_pan_link", "head_tilt_link"],
                                static_joint_states=neck_static_joints)
        self.chains["neck"] = neck
        # Arms
        r_joints = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint",
                    "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint",
                    "r_wrist_roll_joint"]
        l_joints = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
                    "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint",
                    "l_wrist_roll_joint"]
        r_links = ["r_shoulder_pan_link", "r_shoulder_lift_link", "r_upper_arm_roll_link",
                   "r_upper_arm_link", "r_elbow_flex_link", "r_forearm_roll_link",
                   "r_forearm_link", "r_wrist_flex_link", "r_wrist_roll_link",
                   "r_gripper_led_frame", "r_gripper_motor_accelerometer_link", "r_gripper_tool_frame",
                   "r_gripper_motor_slider_link", "r_gripper_motor_screw_link"]
        l_links = ["l_shoulder_pan_link", "l_shoulder_lift_link", "l_upper_arm_roll_link",
                   "l_upper_arm_link", "l_elbow_flex_link", "l_forearm_roll_link", "l_forearm_link",
                   "l_wrist_flex_link", "l_wrist_roll_link", "l_gripper_led_frame",
                   "l_gripper_motor_accelerometer_link", "l_gripper_tool_frame",
                   "l_gripper_motor_slider_link", "l_gripper_motor_screw_link",
                   "l_force_torque_link", "l_force_torque_adapter_link"]
        r_gripper_joints = ["r_gripper_l_finger_joint", "r_gripper_r_finger_joint"]
        r_gripper_links = ["r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link",
                           "r_gripper_l_finger_link", "r_gripper_r_finger_link",
                           "r_gripper_l_finger_tip_frame", "r_gripper_palm_link"]
        l_gripper_joints = ["l_gripper_l_finger_joint", "l_gripper_r_finger_joint"]
        l_gripper_links = ["l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link",
                           "l_gripper_l_finger_link", "l_gripper_r_finger_link",
                           "l_gripper_l_finger_tip_frame", "l_gripper_palm_link"]
        # Arms
        left = ChainDescription("left", l_joints, l_links)
        right = ChainDescription("right", r_joints, r_links)
        left_inter = InteractionDescription(left, "l_wrist_roll_link")
        right_inter = InteractionDescription(right, "r_wrist_roll_link")
        # Gripper
        left_gripper = GripperDescription("left_gripper", l_gripper_links, l_gripper_joints,
                                          gripper_meter_to_jnt_multiplier=5.0,
                                          gripper_minimal_position=0.013,
                                          gripper_convergence_delta=0.005)
        right_gripper = GripperDescription("right_gripper", r_gripper_links, r_gripper_joints,
                                           gripper_meter_to_jnt_multiplier=5.0,
                                           gripper_minimal_position=0.013,
                                           gripper_convergence_delta=0.005)
        # Adding Arm + Gripper
        left_arm = ManipulatorDescription(left_inter, tool_frame="l_gripper_tool_frame",
                                          gripper_description=left_gripper)
        right_arm = ManipulatorDescription(right_inter, tool_frame="r_gripper_tool_frame",
                                           gripper_description=right_gripper)
        self.chains["left"] = left_arm
        self.chains["right"] = right_arm
        # Adding Static Joint Poses
        # Static Arm Positions
        r_arm_park = [-1.712, -0.256, -1.463, -2.12, 1.766, -0.07, 0.051]
        l_arm_park = [1.712, -0.264, 1.38, -2.12, 16.996, -0.073, 0.0]
        self.add_static_joint_chain("right", "park", r_arm_park)
        self.add_static_joint_chain("left", "park", l_arm_park)
        # Static Gripper Positions
        gripper_confs = {"open": [0.548, 0.548],
                         "close": [0.0, 0.0]}
        self.add_static_gripper_chains("left", gripper_confs)
        self.add_static_gripper_chains("right", gripper_confs)

    def get_camera_frame(self, name="optical_frame"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)


class InitializedRobotDescription():
    # singleton instance short named as 'i'
    i = None
    current_description_loaded = None

    def __init__(self, robot_description):
        assert issubclass(robot_description, RobotDescription) and robot_description is not RobotDescription
        if not InitializedRobotDescription.i or \
                InitializedRobotDescription.current_description_loaded is not robot_description:
            InitializedRobotDescription.current_description_loaded = robot_description
            InitializedRobotDescription.i = robot_description()


InitializedRobotDescription(PR2Description)
print("loaded robot description")
#print(InitializedRobotDescription.i.get_static_gripper_chain("left", "close").items())
