from copy import deepcopy
from numbers import Number
import logging
import re

from ros.rosbridge import ros_client


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

    def add_static_joint_chains(self, static_joint_states):
        """
        This function adds static joints chains, which are given in a dictionary.

        :type static_joint_states: dict[str: list[float]]
        :param static_joint_states: Static joint chains, where keys hold the configuration name
                                    and values hold a list of floats/joint configurations.
        """
        for configuration, joint_states in static_joint_states.items():
            if not self.add_static_joint_chain(configuration, joint_states):
                logging.error("(robot_description) Could not add all static_joint_states for the chain %s.", self.name)
                break

    def add_static_joint_chain(self, configuration, static_joint_states):
        """
        This function adds one static joints chain with the given configuration name.

        :param configuration: Configuration name of the static joint configuration
        :type configuration: str
        :param static_joint_states: List of floats/joint configurations
        :type static_joint_states: list[float]
        """
        if all(map(lambda n: isinstance(n, Number), static_joint_states)):
            if len(static_joint_states) == len(self.joints):
                configurations = list(self.static_joint_states.keys())
                if configuration not in configurations:
                    self.static_joint_states = self.static_joint_states
                    self.static_joint_states[configuration] = static_joint_states
                    return True
                else:
                    logging.warning("(robot_description) The chain %s already has static joint values "
                            "for the config %s.", self.name, configuration)
            else:
                logging.error("(robot_description) The number of the joint values does not equal the amount of the joints"
                       "for the chain %s.", self.name)

    def get_static_joint_chain(self, configuration):
        """
        Gets a static joint chain as dictionary of its joint names and values.

        :param configuration: Name of the configuration
        :type configuration: str
        :return: dict[str: float]
        """
        try:
            joint_values = self.static_joint_states[configuration]
        except KeyError:
            logging.error("(robot_description) Configuration %s is unknown for the chain %s.", configuration, self.name)
            return None
        return dict(zip(self.joints, joint_values))


class GripperDescription(ChainDescription):
    """
    This class represents a gripper of a robot. It allows to specify more parameters
    for the robots gripper and set static gripper configurations (since it inherits
    from ChainDescription).
    """

    def __init__(self, name, gripper_links=None, gripper_joints=None,
                 static_gripper_joint_states=None,
                 gripper_meter_to_jnt_multiplier=1.0, gripper_minimal_position=0.0,
                 gripper_convergence_delta=0.001):
        """
        :type gripper_links: list[str]
        :type gripper_joints: list[str]
        :type static_gripper_joint_states: dict[str: list[float]]
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
        super().__init__(tmp.name, tmp.joints, tmp.links, base_link=tmp.base_link,
                         static_joint_states=tmp.static_joint_states)
        self.chain_description = tmp
        self.eef_link = eef_link


class ManipulatorDescription(InteractionDescription):
    """
    This class allows with the given interaction description to include a gripper
    description which is placed between the last link of the interaction description
    and the rest of it.
    Independently from that a tool frame can be saved, which allows to use objects
    to manipulate the environment.
                                                                           |--> (tool_frame)
        chain of links: interaction_description <-> (gripper_description) -|
                                                                           |--> eef_link
    """

    def __init__(self, interaction_description: InteractionDescription,
                 tool_frame: str = None, gripper_description: GripperDescription = None):
        tmp = deepcopy(interaction_description)
        super().__init__(tmp.chain_description, tmp.eef_link)
        self.interaction_description = tmp
        self.tool_frame = tool_frame
        self.gripper = gripper_description


class CameraDescription:
    """
    This class represents a camera by saving the camera link from the URDF and other
    parameters specifically for that camera.
    """

    def __init__(self, frame: str,
                 minimal_height=0.0, maximal_height=0.0,
                 vertical_angle=0.0, horizontal_angle=0.0,
                 other_params=None):
        """
        :type other_params: dict[str: float]
        """
        self.frame = frame
        self.min_height = minimal_height
        self.max_height = maximal_height
        self.vertical_angle = vertical_angle
        self.horizontal_angle = horizontal_angle
        # static, but flexible params
        self.params = other_params if other_params else {}

class GraspingDescription():
    """
    This class represents all possible grasp a robot can perform and the grasps
    the robot can perform for a specific object.
    """
    def __init__(self, grasp_dir=None):
        self.grasps = grasp_dir if grasp_dir else {}
        self.grasps_for_object = {}

    def add_grasp(self, grasp, orientation):
        """
        Adds a Grasping side like "top", "left", "front" and the corresponding orientation
        of the gripper to this description.
        :param grasp: The type of grasp like "top" or "left"
        :param orientation: The orientation the Gripper has to have in order to
        achive this grasp. In world coordinate frame
        """
        self.grasps[grasp] = orientation

    def add_graspings_for_object(self, grasps, object):
        """
        Adds all possible Grasps for the specified object. The used grasps have to
        be registered beforehand via the add_grasp method.
        :param graps: A list of all graps for this object.
        :param object: The object for which the grasps should be specified
        """
        self.grasps_for_object[object] = grasps

    def get_all_grasps(self):
        return self.grasps.keys()

    def get_orientation_for_grasp(self, grasp):
        return self.grasps[grasp]

    def get_grasps_for_object(self, object):
        return self.grasps_for_object[object]

class RobotDescription:
    """
    The RobotDescription as an abstract class which needs to be inherited from to implement
    a robot description for specific object. It implements different functions to add and get
    chains of different objects types which inherit of the class ChainDescription. Moreover,
    it allows to model the robot with its odom_frame, base_frame, base_link and torso links
    and joints. Different cameras can be added and static transforms and poses can be added too.
    """

    def __init__(self, name, base_frame, base_link, ik_joints, torso_link=None, torso_joint=None,
                 odom_frame=None, odom_joints=None):
        """
        Initialises the robot description with the given frames.

        :type name: str
        :type odom_frame: str
        :type odom_frame: [str]
        :type base_frame: str
        :type base_link: str
        :type torso_link: str
        :type torso_joint: str
        :type ik_joints: [str]
        """
        self.name = name
        self.chains = {}  # dict{str: ChainDescription}
        self.cameras = {}  # dict{str: CameraDescription}
        self.static_transforms = []  # list[tf]
        self.static_poses = []  # list[pose]
        self.odom_frame = odom_frame
        self.odom_joints = odom_joints if odom_joints else []  # list[str]
        self.base_frame = base_frame
        self.base_link = base_link
        self.torso_link = torso_link
        self.torso_joint = torso_joint

    def _safely_access_chains(self, chain_name, verbose=True):
        """
        This function returns the chain_description of the name chain_name or None, if there
        exists no chain description with the name chain_name.
        """
        try:
            chain_description = self.chains[chain_name]
        except KeyError:
            if verbose:
                logging.warning("(robot_description) Chain name %s is unknown.", chain_name)
            return None
        return chain_description

    def _get_chain_description(self, chain_name, description_type=ChainDescription, is_same_description_type=True):
        """
        This function checks if there is a chain saved in self.chains with the chain name chain_name.
        Moreover, if is_same_description_type is True it will be checked if the found chain is of the given
        object class description_type. If is_same_description_type is False, it will be just checked if
        the found chain is a subclass of description_type.

        :return: subclass of ChainDescription or None
        """
        if issubclass(description_type, ChainDescription):
            chain_description = self._safely_access_chains(chain_name)
            if not is_same_description_type:
                return chain_description
            else:
                if type(chain_description) is description_type:
                    return chain_description
                else:
                    logging.error("(robot_description) The chain %s is not of type %s, but of type %s.",
                           chain_name, description_type, type(chain_description))
        else:
            logging.warning("(robot_description) Only subclasses of ChainDescription are allowed.")

    def get_tool_frame(self, manipulator_name: str):
        """
        Returns the tool frame of the manipulator description with the name manipulator name.

        :return: str
        """
        manipulator_description = self._get_chain_description(manipulator_name, description_type=ManipulatorDescription)
        if manipulator_description:
            return manipulator_description.tool_frame
        else:
            logging.error("(robot_description) Could not get the tool frame of the manipulator %s.", manipulator_name)

    def get_static_joint_chain(self, chain_name: str, configuration: str):
        """
        Returns the static joint chain given the chains name chain_name and the configurations name configuration.

        :return: dict[str: float]
        """
        chain_description = self._get_chain_description(chain_name, is_same_description_type=False)
        if chain_description:
            return chain_description.get_static_joint_chain(configuration)
        else:
            logging.error("(robot_description) Could not get static joint chain called %s of the chain %s.",
                   configuration, chain_name)

    def get_static_tf(self, base_link: str, target_link: str):
        pass

    def get_static_pose(self, frame: str):
        pass

    def add_chain(self, name: str, chain_description: ChainDescription):
        """
        This functions adds the chain description chain_description with the name name and
        overwrites the existing chain description of name name, if it already exists in self.chains.
        """
        if issubclass(type(chain_description), ChainDescription):
            if self._safely_access_chains(name, verbose=False):
                logging.warning("(robot_description) Replacing the chain description of the name %s.", name)
            self.chains[name] = chain_description
            return True
        else:
            logging.error("(robot_description) Given chain_description object is no subclass of ChainDescription.")

    def add_chains(self, chains_dict):
        """
        This function calls recursively the self.add_chain function and adds therefore the chain description
        saved in the values part of the dictionary chains_dict with the names saved in the key part of the
        dictionary chains_dict.

        :type chains_dict: dict[str: ChainDescription]
        """
        for name, chain in chains_dict.items():
            if not self.add_chain(name, chain):
                logging.error("(robot_description) Could not add the chain object of name %s.", name)
                break

    def add_camera(self, name: str, camera_description: CameraDescription):
        """
        This functions adds the camera description camera_description with the name name and
        overwrites the existing camera description of name name, if it already exists in self.cameras.
        """
        if type(camera_description) is CameraDescription:
            found = True
            try:
                self.cameras[name]
            except KeyError:
                found = False
            if found:
                logging.warning("(robot_description) Replacing the camera description of the name %s.", name)
            self.cameras[name] = camera_description
            return True
        else:
            logging.error("(robot_description) Given camera_description object is not of type CameraDescription.")

    def add_cameras(self, cameras_dict):
        """
        This function calls recursively the self.add_camera function and adds therefore the camera description
        saved in the values part of the dictionary cameras_dict with the names saved in the key part of the
        dictionary cameras_dict.

        :type cameras_dict: dict[str: CamerasDescription]
        """
        for name, camera in cameras_dict.items():
            if not self.add_camera(name, camera):
                logging.error("(robot_description) Could not add the camera object of name %s.", name)
                break

    def get_camera_frame(self, camera_name):
        """
        Returns the camera frame of the given camera with the name camera_name.

        :return: str
        """
        try:
            camera_description = self.cameras[camera_name]
        except KeyError:
            logging.error("(robot_description) Camera name %s is unknown.", camera_name)
            return None
        return camera_description.frame

    def add_static_joint_chain(self, chain_name: str, configuration: str, static_joint_states):
        """
        This function calls the add_static_joint_chain function on the chain object with the name chain_name.
        For more information see the add_static_joint_chain in ChainDescription.

        :type static_joint_states: list[float]
        """
        return self.chains[chain_name].add_static_joint_chain(configuration, static_joint_states)

    def add_static_joint_chains(self, chain_name: str, static_joint_states):
        """
        This function calls recursively the self.add_static_joint_chain function with the name chain_name
        and adds therefore the static joint values saved in the values part of the dictionary static_joint_states
        with the configuration names saved in the key part of the dictionary static_joint_states.

        :type static_joint_states: dict[str: list[float]]
        """
        for configuration, joint_states in static_joint_states.items():
            if not self.add_static_joint_chain(chain_name, configuration, joint_states):
                logging.error("(robot_description) Could not add the static joint chain called %s for chain %s.",
                       configuration, chain_name)
                break

    def add_static_gripper_chain(self, manipulator_name: str, configuration: str, static_joint_states):
        """
        This function adds a static gripper chain to a gripper description if there exists a manipulator description
        with the name manipulator_name. The static gripper joint vales in static_joint_states are then saved
        with the configuration name configuration in the manipulator description object.
        For more information see the add_static_joint_chain in ChainDescription.

        :type static_joint_states: dict[str: list[float]]
        """
        manipulator_description = self._get_chain_description(manipulator_name, ManipulatorDescription)
        if manipulator_description and manipulator_description.gripper:
            return manipulator_description.gripper.add_static_joint_chain(configuration, static_joint_states)

    def add_static_gripper_chains(self, manipulator_name: str, static_joint_states):
        """
        This function calls recursively the self.add_static_gripper_chain function with the name manipulator_name
        and adds therefore the static joint values saved in the values part of the dictionary static_joint_states
        with the configuration names saved in the key part of the dictionary static_joint_states.

        :type static_joint_states: dict[str: list[float]]
        """
        for configuration, joint_states in static_joint_states.items():
            if not self.add_static_gripper_chain(manipulator_name, configuration, joint_states):
                logging.error("(robot_description) Could not add static gripper chain called %s for manipulator chain %s.",
                       configuration, manipulator_name)
                break

    def get_static_gripper_chain(self, manipulator_name: str, configuration: str):
        """
        Returns the static gripper joint chain given the manipulator name manipulator_name and the configuration name configuration.

        For more information see the function get_static_joint_chain in ChainDescription.

        :return: dict[str: list[float]] or None
        """
        manipulator_description = self._get_chain_description(manipulator_name, ManipulatorDescription)
        if manipulator_description and manipulator_description.gripper:
            return manipulator_description.gripper.get_static_joint_chain(configuration)

    # @staticmethod
    # def from_urdf(urdf_path: str):
    #     # URDF Python library does not like ROS package paths --> replace
    #     with tempfile.NamedTemporaryFile(suffix=".urdf") as temp_urdf_file:
    #         with open(urdf_path) as urdf_file:
    #             urdf_resolved = replace_package_urls(urdf_file.read())
    #         temp_urdf_file.write(urdf_resolved)
    #         urdf = URDF.load(temp_urdf_file)
    #     ik_joints = [joint.name for joint in urdf.actuated_joints]


class UR5RobotiqDescription(RobotDescription):
    def __init__(self):
        # all joints which are not fix,
        ik_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                     "wrist_3_joint", "robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint",
                     "robotiq_85_left_finger_joint", "robotiq_85_right_finger_joint",
                     "robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
                     "robotiq_85_left_finger_tip_joint", "robotiq_85_left_finger_tip_joint"]
        super(UR5RobotiqDescription, self).__init__("ur5_robotiq", "world", "base_link", ik_joints)

        # Arm
        arm_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                      "wrist_3_joint"]
        arm_links = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]
        gripper_joints = ["robotiq_85_left_finger_joint", "robotiq_85_right_finger_joint",
                          "robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
                          "robotiq_85_left_finger_tip_joint", "robotiq_85_left_finger_tip_joint"]
        gripper_links = ["robotiq_85_base_link", "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
                         "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
                         "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
                         "robotiq_85_left_finger_tip_link" "robotiq_85_right_finger_tip_link"]

        # Arm
        manipulator_chain = ChainDescription("manipulator", arm_joints, arm_links)
        manipulator_inter = InteractionDescription(manipulator_chain, "ee_link")
        # Gripper
        gripper = GripperDescription("gripper", gripper_links, gripper_joints)
        # Adding Arm + Gripper
        manipulator = ManipulatorDescription(manipulator_inter, tool_frame="ee_link",
                                             gripper_description=gripper)
        self.add_chains({"manipulator": manipulator})
        # Adding Static Joint Poses
        # Static Arm Positions
        manipulator_home = [0, 0, 0, 0, 0, 0]
        self.add_static_joint_chain("manipulator", "home", manipulator_home)
        # Static Gripper Positions
        gripper_confs = {"open": [0.0], "close": [1.0]}
        self.add_static_gripper_chains("gripper", gripper_confs)


class PR2Description(RobotDescription):

    def __init__(self):


        super().__init__("pr2", "base_footprint", "base_link", "torso_lift_link", "torso_lift_joint")
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
        self.add_cameras({"optical_frame": wide_stereo_optical_frame, "kinect": kinect,
                            "openni": openni, "stereo": stereo})

        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]
        # Chains
        neck_static_joints = {}
        neck_static_joints["forward"] = [0.0, 0.0]
        neck = ChainDescription("neck", ["head_pan_joint", "head_tilt_joint"],
                                ["head_pan_link", "head_tilt_link"],
                                static_joint_states=neck_static_joints)
        self.add_chain("neck", neck)
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
        self.add_chains({"left": left_arm, "right": right_arm})
        # Adding Static Joint Poses
        # Static Arm Positions
        r_arm_park = [-1.712, -0.256, -1.463, -2.12, 1.766, -0.07, 0.051]
        l_arm_park = [1.712, -0.264, 1.38, -2.12, 16.996, -0.073, 0.0]
        self.add_static_joint_chain("right", "park", r_arm_park)
        self.add_static_joint_chain("left", "park", l_arm_park)
        # Static Gripper Positions
        gripper_confs = {"open": [0.548, 0.548], "close": [0.0, 0.0]}
        self.add_static_gripper_chains("left", gripper_confs)
        self.add_static_gripper_chains("right", gripper_confs)

        self.grasps = GraspingDescription({"front": [0, 0, 0, 1],
                                        "left": [0, 0, -1, 1],
                                        "right": [0, 0, 1, 1],
                                        "top": [0, 1, 0, 1]})

    def get_camera_frame(self, name="optical_frame"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)


class BoxyDescription(RobotDescription):

    def __init__(self):

        super().__init__("boxy", "base_footprint", "base_link", "triangle_base_link",
                         "triangle_base_joint", odom_frame="odom",
                         odom_joints=["odom_x_joint", "odom_y_joint", "odom_z_joint"])
        camera = CameraDescription("head_mount_kinect2_rgb_optical_frame",
                                   minimal_height=0.0, maximal_height=2.5,
                                   horizontal_angle=0.99483, vertical_angle=0.75049)
        self.add_camera("camera", camera)
        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]
        # Neck
        neck_links = ["neck_shoulder_link", "neck_upper_arm_link", "neck_forearm_link",
                      "neck_wrist_1_link", "neck_wrist_2_link", "neck_wrist_3_link"]
        neck_joints = ["neck_shoulder_pan_joint", "neck_shoulder_lift_joint", "neck_elbow_joint",
                       "neck_wrist_1_joint", "neck_wrist_2_joint", "neck_wrist_3_joint"]
        neck_base = "neck_base_link"
        neck_away = [-1.3155, -1.181355, -1.9562, 0.142417, 1.13492, 0.143467]  # up
        neck_down = [-1.176, -3.1252, -0.8397, 0.83967, 1.1347, -0.0266]
        neck_down_left = [-0.776, -3.1252, -0.8397, 0.83967, 1.1347, -0.0266]
        neck_down_right = [-2.176, -3.1252, -0.8397, 0.83967, 1.1347, -0.0266]
        neck_behind_up = [2.10, -1.11, -1.89, 0.64, 1.11, -0.43]
        neck_behind = [-1.68, -0.26, -0.12, -0.24, 1.49, 3.09]
        # neck_forward = [-1.39, -3.09, -0.78, 1.91, 1.51, -0.14]
        neck_forward = [-1.20, 2.39, 0.17, 1.77, 1.44, -0.35]
        neck_right = [-1.20, 2.39, 0.17, 1.77, 1.44, 0.0]
        neck_left = [-1.20, 2.39, 0.17, 1.77, 1.44, -0.7]
        neck = ChainDescription("neck", neck_joints, neck_links, base_link=neck_base)
        neck.add_static_joint_chains({"away": neck_away, "down": neck_down, "down_left": neck_down_left,
                                      "down_right": neck_down_right, "behind_up": neck_behind_up, "behind": neck_behind,
                                      "forward": neck_forward, "right": neck_right, "left": neck_left})
        self.add_chain("neck", neck)
        # Arms
        left_joints = ["left_arm_0_joint", "left_arm_1_joint", "left_arm_2_joint",
                       "left_arm_3_joint", "left_arm_4_joint", "left_arm_5_joint",
                       "left_arm_6_joint"]
        left_links = ["left_arm_1_link", "left_arm_2_link", "left_arm_3_link",
                      "left_arm_4_link", "left_arm_5_link", "left_arm_6_link",
                      "left_arm_7_link"]
        right_joints = ["right_arm_0_joint", "right_arm_1_joint", "right_arm_2_joint",
                        "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint",
                        "right_arm_6_joint"]
        right_links = ["right_arm_1_link", "right_arm_2_link", "right_arm_3_link",
                       "right_arm_4_link", "right_arm_5_link", "right_arm_6_link",
                       "right_arm_7_link"]
        # Gripper
        gripper_meter_to_jnt_multiplier = 1.0
        gripper_minimal_position = 0.0
        gripper_convergence_delta = 0.001
        l_gripper_links = ["left_gripper_base_link", "left_gripper_finger_left_link",
                           "left_gripper_finger_right_link", "left_gripper_gripper_left_link",
                           "left_gripper_gripper_right_link", "left_gripper_tool_frame"]
        l_gripper_joints = ["left_gripper_joint"]
        left_gripper = GripperDescription("left_gripper", gripper_links=l_gripper_links,
                                          gripper_joints=l_gripper_joints,
                                          gripper_meter_to_jnt_multiplier=gripper_meter_to_jnt_multiplier,
                                          gripper_minimal_position=gripper_minimal_position,
                                          gripper_convergence_delta=gripper_convergence_delta)
        r_gripper_links = ["right_gripper_base_link", "right_gripper_finger_left_link",
                           "right_gripper_finger_right_link", "right_gripper_gripper_left_link",
                           "right_gripper_gripper_right_link", "right_gripper_tool_frame"]
        r_gripper_joints = ["right_gripper_joint"]
        right_gripper = GripperDescription("right_gripper", gripper_links=r_gripper_links,
                                           gripper_joints=r_gripper_joints,
                                           gripper_meter_to_jnt_multiplier=gripper_meter_to_jnt_multiplier,
                                           gripper_minimal_position=gripper_minimal_position,
                                           gripper_convergence_delta=gripper_convergence_delta)
        # Arms
        left = ChainDescription("left", left_joints, left_links)
        left_inter = InteractionDescription(left, "left_arm_7_link")
        left_arm = ManipulatorDescription(left_inter, tool_frame="left_gripper_tool_frame",
                                          gripper_description=left_gripper)
        right = ChainDescription("right", right_joints, right_links)
        right_inter = InteractionDescription(right, "right_arm_7_link")
        right_arm = ManipulatorDescription(right_inter, tool_frame="right_gripper_tool_frame",
                                           gripper_description=right_gripper)
        self.add_chains({"left": left_arm, "right": right_arm})
        # Adding Static Joint Poses
        # Static Arm Positions
        l_carry = [-1.858, 0.70571, 0.9614, -0.602, -2.5922, -1.94065, -1.28735]
        r_carry = [1.858, -0.70571, -0.9614, 0.602, 2.5922, 1.94065, 1.28735]
        l_handover = [-0.32, 1.8, -0.74, -1.49, 2.29, 1.68, 0.2]
        l_flip = [-1.2274070978164673, 0.8496202230453491, -0.10349386930465698,
                  -1.0852965116500854, -0.4587952196598053, 1.259474515914917,
                  -0.06962397694587708]
        left_arm.add_static_joint_chains({"park": l_carry, "handover": l_handover, "flip": l_flip})
        right_arm.add_static_joint_chain("park", r_carry)

        # Grasping Poses
        self.grasps = GraspingDescription({"left":[1,0,0,1],
                                        "top": [1,1,0,0],
                                        "right": [0,1,1,0],
                                        "front": [1,0,1,0]})



    def get_camera_frame(self, name="camera"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)


class DonbotDescription(RobotDescription):

    def __init__(self):

        super().__init__("donbot", "base_footprint", "base_link", "ur5_base_link",
                         "arm_base_mounting_joint", odom_frame="odom",
                         odom_joints=["odom_x_joint", "odom_y_joint", "odom_z_joint"])
        # Camera
        rgb_camera = CameraDescription("camera_link",
                                       horizontal_angle=0.99483, vertical_angle=0.75049,
                                       minimal_height=0.5, maximal_height=1.2)
        rs_camera = CameraDescription("rs_camera_link",
                                      horizontal_angle=0.99483, vertical_angle=0.75049,
                                      minimal_height=0.5, maximal_height=1.2)
        # not in the donbot.urdf, although used in cram
        # realsense_camera = CameraDescription("rs_camera_depth_optical_frame",
        #                                     horizontal_angle=0.99483, vertical_angle=0.75049,
        #                                     minimal_height=0.5, maximal_height=1.2)
        # virtual_camera = CameraDescription("rs_camera_color_optical_frame",
        #                                   horizontal_angle=0.99483, vertical_angle=0.75049,
        #                                   minimal_height=0.5, maximal_height=1.2)
        self.add_cameras({"rgb_camera": rgb_camera, "rs_camera": rs_camera})
        # self.cameras["realsense_camera"] = realsense_camera
        # self.cameras["virtual_camera"] = virtual_camera
        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]

        # Gripper
        gripper_links = ["wrist_collision", "gripper_base_link", "gripper_finger_left_link",
                         "gripper_finger_right_link", "gripper_gripper_left_link", "gripper_gripper_right_link",
                         "marco_left_finger_link", "marco_right_finger_link"]
        gripper_joints = ["gripper_joint"]
        gripper = GripperDescription("gripper", gripper_links=gripper_links, gripper_joints=gripper_joints,
                                     gripper_meter_to_jnt_multiplier=1.0, gripper_minimal_position=0.0,
                                     gripper_convergence_delta=0.001)
        # Arm
        arm_links = ["ur5_shoulder_link", "ur5_upper_arm_link", "ur5_forearm_link", "ur5_wrist_1_link",
                     "ur5_wrist_2_link", "ur5_wrist_3_link"]
        arm_joints = ["ur5_shoulder_pan_joint", "ur5_shoulder_lift_joint", "ur5_elbow_joint",
                      "ur5_wrist_1_joint", "ur5_wrist_2_joint", "ur5_wrist_3_joint"]
        arm_chain = ChainDescription("left", arm_joints, arm_links)
        arm_inter = InteractionDescription(arm_chain, "ur5_ee_link")
        arm_manip = ManipulatorDescription(arm_inter, tool_frame="gripper_tool_frame",
                                           gripper_description=gripper)  # or ur5_tool0
        self.add_chain("left", arm_manip)
        # Neck
        neck_base_link = "ur5_base_link"
        neck = ChainDescription("neck", arm_joints, arm_links, base_link=neck_base_link)
        self.add_chain("neck", neck)
        # Adding Static Joint Poses
        # Static Neck Positions
        down = [4.130944728851318, 0.04936718940734863, -1.9734209219561976,
                -1.7624157110797327, 1.6369260549545288, -1.6503327528582972]
        right = [1.6281344890594482, -1.1734271208392542, -1.1555221716510218,
                 # 2:-1.4734271208392542, 4: -0.9881671110736292,
                 -1.7555221716510218, 1.4996352195739746, -1.4276765028582972]
        left = [-1.6281344890594482, -1.1734271208392542, -1.1555221716510218,
                # 2:-1.4734271208392542, 4: -0.9881671110736292,
                -1.7555221716510218, 1.4996352195739746, -1.4276765028582972]
        right_separators = [1.4752850532531738, -1.4380276838885706, -1.9198325316058558,
                            -0.0680769125567835, 1.704722285270691, -1.5686963240252894]
        right_separators_preplace_state = [1.585883378982544, -0.49957687059511,
                                           -1.61414081255068, -1.1720898787127894,
                                           1.37771737575531, -1.3602331320392054]
        self.add_static_joint_chains("neck", {"down": down, "right": right, "left": left,
                                              "right_separators": right_separators,
                                              "right_separators_preplace_state": right_separators_preplace_state})
        # Static Arm Positions
        park = [3.234022855758667, -1.5068710486041468, -0.7870314756976526, -2.337625328694479,
                1.5699548721313477, -1.6504042784320276]
        self.add_static_joint_chain("left", "park", park)

    def get_camera_frame(self, name="rgb_camera"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)


class HSRDescription(RobotDescription):

    def __init__(self):

        super().__init__("hsr", "base_footprint", "base_link", "arm_lift_link", "arm_lift_joint")
        # Camera
        head_center_camera = CameraDescription("head_center_camera_frame",
                                               horizontal_angle=0.99483, vertical_angle=0.75049)
        head_r_camera = CameraDescription("head_r_stereo_camera_link",
                                          horizontal_angle=0.99483, vertical_angle=0.75049)
        head_l_camera = CameraDescription("head_r_stereo_camera_link",
                                          horizontal_angle=0.99483, vertical_angle=0.75049)
        head_rgbd_camera = CameraDescription("head_rgbd_sensor_link",
                                             horizontal_angle=0.99483, vertical_angle=0.75049)
        hand_camera = CameraDescription("hand_camera_frame",
                                        horizontal_angle=0.99483, vertical_angle=0.75049)
        self.add_cameras({"head_center_camera": head_center_camera, "head_rgbd_camera": head_rgbd_camera,
                          "head_l_camera": head_l_camera, "head_r_camera": head_r_camera,
                          "hand_camera": hand_camera})
        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]
        # Neck
        neck_links = ["head_pan_link", "head_tilt_link"]
        neck_joints = ["head_pan_joint", "head_tilt_joint"]
        neck_forward = {"forward": [0.0, 0.0], "down": [0.0, -0.7]}
        neck_chain = ChainDescription("neck", neck_joints, neck_links, static_joint_states=neck_forward)
        self.add_chain("neck", neck_chain)
        # Arm
        arm_joints = ["arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        arm_links = ["arm_flex_link", "arm_roll_link", "wrist_flex_link", "wrist_roll_link"]
        arm_carry = {"park": [0, 1.5, -1.85, 0]}
        gripper_links = ["hand_l_distal_link", "hand_l_spring_proximal_link", "hand_palm_link",
                         "hand_r_distal_link", "hand_r_spring_proximal_link"]
        gripper_joints = ["hand_motor_joint"]
        gripper = GripperDescription("gripper", gripper_links=gripper_links, gripper_joints=gripper_joints,
                                     gripper_meter_to_jnt_multiplier=1.0, gripper_minimal_position=0.0,
                                     gripper_convergence_delta=0.001)
        arm_chain = ChainDescription("left", arm_joints, arm_links, static_joint_states=arm_carry)
        arm_inter = InteractionDescription(arm_chain, "wrist_roll_link")
        arm_manip = ManipulatorDescription(arm_inter, tool_frame="gripper_tool_frame", gripper_description=gripper)
        self.add_chain("left", arm_manip)
        self.add_static_gripper_chains("left", {"open": [0.3], "close": [0.0]})

    def get_camera_frame(self, name="head_center_camera"):
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
            logging.info("(robot-description) (Re)Loaded Description of robot %s.", self.i.name)


def update_robot_description(robot_name=None, from_ros=None):
    # Get robot name
    if robot_name:
        robot = robot_name
    elif from_ros:
        try:
            urdf = ros_client.get_param('robot_description')
        except Exception as e:
            logging.error("(robot-description) Could not get robot name from parameter server. Try again.")
            return None
        res = re.findall(r"robot\ *name\ *=\ *\"\ *[a-zA-Z_1-9]*\ *\"", urdf)
        if len(res) == 1:
            begin = res[0].find("\"")
            end = res[0][begin + 1:].find("\"")
            robot = res[0][begin + 1:begin + 1 + end].lower()
    else:
        return None

    # Choose Description based on robot name
    if 'donbot' in robot:
        description = DonbotDescription
    elif 'pr2' in robot:
        description = PR2Description
    elif 'boxy' in robot:
        description = BoxyDescription
    elif 'hsr' in robot:
        description = HSRDescription
    elif "ur5_robotiq" in robot:
        description = UR5RobotiqDescription
    else:
        logging.error("(robot-description) The given robot name %s has no description class.", robot_name)
        return None
    return InitializedRobotDescription(description)


update_robot_description(from_ros=True)#"ur5_robotiq")#  # todo: put in ros init
