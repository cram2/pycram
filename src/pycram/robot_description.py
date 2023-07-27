import logging
import rospy
import rospkg
from copy import deepcopy
from numbers import Number
from typing import List, Optional, Dict, Union, Type
from urdf_parser_py.urdf import URDF
from . import utils


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class ChainDescription:
    """
    This class saves a kinematic chain by saving the links and the joints.
    Moreover, this class offers to set a base_link and specify static joint states
    (e. g. the "park" state of the right arm).

        chain of links: link1 <-> joint1 <-> link2 <-> joint2 <-> ... <-> link6
    """

    def __init__(self, name: str,
                 joints: List[str],
                 links: List[str],
                 base_link: Optional[str] = None,
                 static_joint_states: Optional[Dict[str, List[float]]] = None):
        """
        :param name: str
        :param joints: list[str]
        :param links: list[str]
        :param static_joint_states: dict[str: list[float]]
        """
        self.name: str = name
        self.base_link: str = base_link
        self.joints: List[str] = joints
        self.links: List[str] = links
        self.static_joint_states: Dict[str: List[float]] = static_joint_states if static_joint_states else {}

    def add_static_joint_chains(self, static_joint_states: Dict[str, List[float]]) -> None:
        """
        This function adds static joints chains, which are given in a dictionary.

        :type static_joint_states: dict[str: list[float]]
        :param static_joint_states: Static joint chains, where keys hold the configuration name
                                    and values hold a list of floats/joint configurations.
        """
        for configuration, joint_states in static_joint_states.items():
            if not self.add_static_joint_chain(configuration, joint_states):
                logger.error("(robot_description) Could not add all static_joint_states for the chain %s.", self.name)
                break

    def add_static_joint_chain(self, configuration: str, static_joint_states: List[float]) -> Union[None, bool]:
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
                    logger.warning("(robot_description) The chain %s already has static joint values "
                            "for the config %s.", self.name, configuration)
            else:
                logger.error("(robot_description) The number of the joint values does not equal the amount of the joints"
                       "for the chain %s.", self.name)

    def get_static_joint_chain(self, configuration: str) -> Union[None, Dict[str, float]]:
        """
        Gets a static joint chain as dictionary of its joint names and values.

        :param configuration: Name of the configuration
        :type configuration: str
        :return: dict[str: float]
        """
        try:
            joint_values = self.static_joint_states[configuration]
        except KeyError:
            logger.error("(robot_description) Configuration %s is unknown for the chain %s.", configuration, self.name)
            return None
        return dict(zip(self.joints, joint_values))


class GripperDescription(ChainDescription):
    """
    This class represents a gripper of a robot. It allows to specify more parameters
    for the robots gripper and set static gripper configurations (since it inherits
    from ChainDescription).
    """

    def __init__(self, name: str,
                 gripper_links: Optional[List[str]] = None,
                 gripper_joints: Optional[List[str]] = None,
                 static_gripper_joint_states: Optional[Dict[str, List[float]]] = None,
                 gripper_meter_to_jnt_multiplier: Optional[float] = 1.0,
                 gripper_minimal_position: Optional[float] = 0.0,
                 gripper_convergence_delta: Optional[float] = 0.001):
        """
        :type gripper_links: list[str]
        :type gripper_joints: list[str]
        :type static_gripper_joint_states: dict[str: list[float]]
        """
        super().__init__(name, gripper_joints, gripper_links, base_link=None,
                         static_joint_states=static_gripper_joint_states)
        # static params
        self.gripper_meter_to_jnt_multiplier: float = gripper_meter_to_jnt_multiplier
        self.gripper_minimal_position: float = gripper_minimal_position
        self.gripper_convergence_delta: float = gripper_convergence_delta


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
        self.chain_description: ChainDescription = tmp
        self.eef_link: str = eef_link


class ManipulatorDescription(InteractionDescription):
    """
    This class allows with the given interaction description to include a gripper
    description which is placed between the last link of the interaction description
    and the rest of it.
    Independently from that a tool frame can be saved, which allows to use objects
    to manipulate the environment::

                                                                           |--> (tool_frame)
        chain of links: interaction_description <-> (gripper_description) -|
                                                                           |--> eef_link
    """

    def __init__(self, interaction_description: InteractionDescription,
                 tool_frame: str = None,
                 gripper_description: GripperDescription = None):
        tmp = deepcopy(interaction_description)
        super().__init__(tmp.chain_description, tmp.eef_link)
        self.interaction_description: InteractionDescription = tmp
        self.tool_frame: str = tool_frame
        self.gripper: GripperDescription = gripper_description


class CameraDescription:
    """
    This class represents a camera by saving the camera link from the URDF and other
    parameters specifically for that camera.
    """

    def __init__(self, frame: str,
                 minimal_height=0.0, maximal_height=0.0,
                 vertical_angle=0.0, horizontal_angle=0.0,
                 other_params: Optional[Dict[str, float]] = None):
        """
        :type other_params: dict[str: float]
        """
        self.frame: str = frame
        self.min_height: float = minimal_height
        self.max_height: float = maximal_height
        self.vertical_angle: float = vertical_angle
        self.horizontal_angle:float = horizontal_angle
        # static, but flexible params
        self.params: Dict[str, float] = other_params if other_params else {}


class GraspingDescription:
    """
    This class represents all possible grasp a robot can perform and the grasps
    the robot can perform for a specific object.
    """
    def __init__(self, grasp_dir: Optional[Dict] = None):
        self.grasps: Dict[str, List[float]] = grasp_dir if grasp_dir else {}
        self.grasps_for_object: Dict['bullet_world.Object', List[str]] = {}

    def add_grasp(self, grasp: str, orientation: List[float]) -> None:
        """
        Adds a Grasping side like "top", "left", "front" and the corresponding orientation
        of the gripper to this description.

        :param grasp: The type of grasp like "top" or "left"
        :param orientation: The orientation the Gripper has to have in order to achive this grasp. In world coordinate frame
        """
        self.grasps[grasp] = orientation

    def add_graspings_for_object(self, grasps: List[str], object: 'bullet_world.Object') -> None:
        """
        Adds all possible Grasps for the specified object. The used grasps have to
        be registered beforehand via the add_grasp method.

        :param grasps: A list of all graps for this object.
        :param object: The object for which the grasps should be specified
        """
        self.grasps_for_object[object] = grasps

    def get_all_grasps(self) -> List[str]:
        return list(self.grasps.keys())

    def get_orientation_for_grasp(self, grasp: str) -> List[float]:
        return self.grasps[grasp]

    def get_grasps_for_object(self, object: 'bullet_world.Object') -> List[str]:
        return self.grasps_for_object[object]


class RobotDescription:
    """
    The RobotDescription as an abstract class which needs to be inherited from to implement
    a robot description for specific object. It implements different functions to add and get
    chains of different objects types which inherit of the class ChainDescription. Moreover,
    it allows to model the robot with its odom_frame, base_frame, base_link and torso links
    and joints. Different cameras can be added and static transforms and poses can be added too.
    """

    def __init__(self, name: str,
                 base_frame: str,
                 base_link: str,
                 torso_link: Optional[str] = None,
                 torso_joint: Optional[str] = None,
                 odom_frame: Optional[str] = None,
                 odom_joints: Optional[List[str]] = None):
        """
        Initialises the robot description with the given frames.

        :type name: str
        :type odom_frame: str
        :type odom_frame: [str]
        :type base_frame: str
        :type base_link: str
        :type torso_link: str
        :type torso_joint: str
        """
        self.name: str = name
        self.chains: Dict[str, Type[ChainDescription]] = {}  # dict{str: ChainDescription}
        self.cameras: Dict[str, CameraDescription] = {}  # dict{str: CameraDescription}
        self.static_transforms: List = []  # list[tf]
        self.static_poses: List = []  # list[pose]
        self.odom_frame: str = odom_frame
        self.odom_joints: List[str] = odom_joints if odom_joints else []  # list[str]
        self.base_frame: str = base_frame
        self.base_link: str = base_link
        self.torso_link: str = torso_link
        self.torso_joint: str = torso_joint

        rospack = rospkg.RosPack()
        filename = rospack.get_path('pycram') + '/resources/' + name + '.urdf'
        with open(filename) as f:
            with utils.suppress_stdout_stderr():
                self.robot_urdf = URDF.from_xml_string(f.read())

    def _safely_access_chains(self, chain_name: str, verbose: Optional[bool] = True) -> Union[None, ChainDescription]:
        """
        This function returns the chain_description of the name chain_name or None, if there
        exists no chain description with the name chain_name.
        """
        try:
            chain_description = self.chains[chain_name]
        except KeyError:
            if verbose:
                logger.warning("(robot_description) Chain name %s is unknown.", chain_name)
            return None
        return chain_description

    def _get_chain_description(self, chain_name: str,
                               description_type: Optional[Type[ChainDescription]] = ChainDescription,
                               is_same_description_type: Optional[bool] = True) -> Union[None, Type[ChainDescription]]:
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
                    logger.error("(robot_description) The chain %s is not of type %s, but of type %s.",
                           chain_name, description_type, type(chain_description))
        else:
            logger.warning("(robot_description) Only subclasses of ChainDescription are allowed.")

    def get_tool_frame(self, manipulator_name: str) -> Union[None, str]:
        """
        Returns the tool frame of the manipulator description with the name manipulator name.

        :return: str
        """
        manipulator_description = self._get_chain_description(manipulator_name, description_type=ManipulatorDescription)
        if manipulator_description:
            return manipulator_description.tool_frame
        else:
            logger.error("(robot_description) Could not get the tool frame of the manipulator %s.", manipulator_name)

    def get_static_joint_chain(self, chain_name: str, configuration: str) -> Union[None, Dict[str, float]]:
        """
        Returns the static joint chain given the chains name chain_name and the configurations name configuration.

        :return: dict[str: float]
        """
        chain_description = self._get_chain_description(chain_name, is_same_description_type=False)
        if chain_description:
            return chain_description.get_static_joint_chain(configuration)
        else:
            logger.error("(robot_description) Could not get static joint chain called %s of the chain %s.",
                   configuration, chain_name)

    def get_static_tf(self, base_link: str, target_link: str):
        pass

    def get_static_pose(self, frame: str):
        pass

    def add_chain(self, name: str, chain_description: ChainDescription) -> Union[None, bool]:
        """
        This functions adds the chain description chain_description with the name name and
        overwrites the existing chain description of name name, if it already exists in self.chains.
        """
        if issubclass(type(chain_description), ChainDescription):
            if self._safely_access_chains(name, verbose=False):
                logger.warning("(robot_description) Replacing the chain description of the name %s.", name)
            self.chains[name] = chain_description
            return True
        else:
            logger.error("(robot_description) Given chain_description object is no subclass of ChainDescription.")

    def add_chains(self, chains_dict: Dict[str, ChainDescription]) -> None:
        """
        This function calls recursively the self.add_chain function and adds therefore the chain description
        saved in the values part of the dictionary chains_dict with the names saved in the key part of the
        dictionary chains_dict.

        :type chains_dict: dict[str: ChainDescription]
        """
        for name, chain in chains_dict.items():
            if not self.add_chain(name, chain):
                logger.error("(robot_description) Could not add the chain object of name %s.", name)
                break

    def add_camera(self, name: str, camera_description: CameraDescription) -> Union[None, bool]:
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
                logger.warning("(robot_description) Replacing the camera description of the name %s.", name)
            self.cameras[name] = camera_description
            return True
        else:
            logger.error("(robot_description) Given camera_description object is not of type CameraDescription.")

    def add_cameras(self, cameras_dict: Dict[str, CameraDescription]) -> None:
        """
        This function calls recursively the self.add_camera function and adds therefore the camera description
        saved in the values part of the dictionary cameras_dict with the names saved in the key part of the
        dictionary cameras_dict.

        :type cameras_dict: dict[str: CamerasDescription]
        """
        for name, camera in cameras_dict.items():
            if not self.add_camera(name, camera):
                logger.error("(robot_description) Could not add the camera object of name %s.", name)
                break

    def get_camera_frame(self, camera_name: str) -> Union[None, str]:
        """
        Returns the camera frame of the given camera with the name camera_name.

        :return: str
        """
        try:
            camera_description = self.cameras[camera_name]
        except KeyError:
            logger.error("(robot_description) Camera name %s is unknown.", camera_name)
            return None
        return camera_description.frame

    def add_static_joint_chain(self,
                               chain_name: str,
                               configuration: str,
                               static_joint_states: List[float]) -> Union[None, bool]:
        """
        This function calls the add_static_joint_chain function on the chain object with the name chain_name.
        For more information see the add_static_joint_chain in ChainDescription.

        :param chain_name: The name of the new static joint chain
        :param configuration: The name of the configuration of this joint chain
        :type static_joint_states: list[float]
        """
        return self.chains[chain_name].add_static_joint_chain(configuration, static_joint_states)

    def add_static_joint_chains(self, chain_name: str, static_joint_states: Dict[str, List[float]]) -> None:
        """
        This function calls recursively the self.add_static_joint_chain function with the name chain_name
        and adds therefore the static joint values saved in the values part of the dictionary static_joint_states
        with the configuration names saved in the key part of the dictionary static_joint_states.

        :param chain_name: The name for the new chain
        :type static_joint_states: dict[str: list[float]]
        """
        for configuration, joint_states in static_joint_states.items():
            if not self.add_static_joint_chain(chain_name, configuration, joint_states):
                logger.error("(robot_description) Could not add the static joint chain called %s for chain %s.",
                       configuration, chain_name)
                break

    def add_static_gripper_chain(self,
                                 manipulator_name: str,
                                 configuration: str,
                                 static_joint_states: Dict[str, List[float]]) -> Union[None, bool]:
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

    def add_static_gripper_chains(self, manipulator_name: str, static_joint_states: Dict[str, List[float]]) -> None:
        """
        This function calls recursively the self.add_static_gripper_chain function with the name manipulator_name
        and adds therefore the static joint values saved in the values part of the dictionary static_joint_states
        with the configuration names saved in the key part of the dictionary static_joint_states.

        :type static_joint_states: dict[str: list[float]]
        """
        for configuration, joint_states in static_joint_states.items():
            if not self.add_static_gripper_chain(manipulator_name, configuration, joint_states):
                logger.error("(robot_description) Could not add static gripper chain called %s for manipulator chain %s.",
                       configuration, manipulator_name)
                break

    def get_static_gripper_chain(self, manipulator_name: str, configuration: str) -> Dict[str, float]:
        """
        Returns the static gripper joint chain given the manipulator name manipulator_name and the configuration name configuration.

        For more information see the function get_static_joint_chain in ChainDescription.

        :return: dict[str: list[float]] or None
        """
        manipulator_description = self._get_chain_description(manipulator_name, ManipulatorDescription)
        if manipulator_description and manipulator_description.gripper:
            return manipulator_description.gripper.get_static_joint_chain(configuration)

    def get_child(self, name):
        """
        Returns the child of a Joint or Link in the URDF. If 'name' is a Joint a
        Link will be returned and vice versa.

        :param name: The name of the Joint/Link for which the child will be returned.
        :return: The child Joint/Link
        """
        if name in self.robot_urdf.joint_map.keys():
            return self.robot_urdf.joint_map[name].child
        elif name in self.robot_urdf.link_map.keys():
            return self.robot_urdf.link_map[name].child
        else:
            rospy.logerr(f"The name: {name} is not part of this robot URDF")

    def get_parent(self, name):
        """
        Returns the parent of a Joint or Link in the URDF. If 'name' is a Joint a
        Link will be returned and vice versa.

        :param name: The name of the Joint/Link for which the parent will be returned.
        :return: The parent Joint/Link
        """
        if name in self.robot_urdf.joint_map.keys():
            return self.robot_urdf.joint_map[name].parent
        elif name in self.robot_urdf.link_map.keys():
            return self.robot_urdf.link_map[name].parent
        else:
            rospy.logerr(f"The name: {name} is not part of this robot URDF")

    # @staticmethod
    # def from_urdf(urdf_path: str):
    #     # URDF Python library does not like ROS package paths --> replace
    #     with tempfile.NamedTemporaryFile(suffix=".urdf") as temp_urdf_file:
    #         with open(urdf_path) as urdf_file:
    #             urdf_resolved = replace_package_urls(urdf_file.read())
    #         temp_urdf_file.write(urdf_resolved)
    #         urdf = URDF.load(temp_urdf_file)
    #     ik_joints = [joint.name for joint in urdf.actuated_joints]
