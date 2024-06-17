# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import rospy
from typing_extensions import List, Dict
from urdf_parser_py.urdf import URDF

from .utils import suppress_stdout_stderr


class RobotDescriptionManager:
    """
    Singleton class to manage multiple robot descriptions. Stores all robot descriptions and loads a robot description
    according to the name of the loaded robot.
    """
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        """
        Initialize the RobotDescriptionManager, if no instance exists a new instance is created.
        """
        if self._initialized: return
        self.descriptions = {}
        self._initialized = True

    def load_description(self, name: str):
        """
        Loads a robot description according to the name of the robot. This required that a robot description with the
        corresponding name is registered.

        :param name: Name of the robot to which the description should be loaded.
        :return: The loaded robot description.
        """
        if name in self.descriptions.keys():
            RobotDescription.current_robot_description = self.descriptions[name]
            return self.descriptions[name]
        else:
            rospy.logerr(f"Robot description {name} not found")

    def register_description(self, description: RobotDescription):
        """
        Register a robot description to the RobotDescriptionManager. The description is stored with the name of the
        description as key. This will later be used to load the description.

        :param description: RobotDescription to register.
        """
        if description.name in self.descriptions.keys():
            raise ValueError(f"Description {description.name} already exists")
        self.descriptions[description.name] = description


class RobotDescription:
    """
    Base class of a robot description. Contains all necessary information about a robot, like the URDF, the base link,
    the torso link and joint, the kinematic chains and cameras.
    """
    current_robot_description = None

    def __init__(self, name: str, base_link: str, torso_link: str, torso_joint: str, urdf_path: str):
        """
        Initialize the RobotDescription. The URDF is loaded from the given path and used as basis for the kinematic
        chains.

        :param name: Name of the robot
        :param base_link: Base link of the robot, meaning the first link in the URDF
        :param torso_link: Torso link of the robot
        :param torso_joint: Torso joint of the robot, this is the joint that moves the torso upwards if there is one
        :param urdf_path: Path to the URDF file of the robot
        """
        self.name = name
        self.base_link = base_link
        self.torso_link = torso_link
        self.torso_joint = torso_joint
        with suppress_stdout_stderr():
            self.urdf_object = URDF.from_xml_file(urdf_path)
        self.kinematic_chains = {}
        self.cameras = {}

    def add_kinematic_chain_description(self, chain: KinematicChainDescription):
        """
        Adds a KinematicChainDescription object to the RobotDescription. The chain is stored with the name of the chain
        as key.

        :param chain: KinematicChainDescription object to add
        """
        if chain.name in self.kinematic_chains.keys():
            raise ValueError(f"Chain {chain.name} already exists for robot {self.name}")
        self.kinematic_chains[chain.name] = chain

    def add_kinematic_chain(self, name: str, start_link: str, end_link: str):
        """
        Creates and adds a KinematicChainDescription object to the RobotDescription.

        :param name: Name of the KinematicChainDescription object
        :param start_link: First link of the chain
        :param end_link: Last link of the chain
        """
        if name in self.kinematic_chains.keys():
            raise ValueError(f"Chain {name} already exists for robot {self.name}")
        chain = KinematicChainDescription(name, start_link, end_link, self.urdf_object)
        self.add_kinematic_chain_description(chain)

    def add_camera_description(self, camera: CameraDescription):
        """
        Adds a CameraDescription object to the RobotDescription. The camera is stored with the name of the camera as key.
        :param camera: The CameraDescription object to add
        """
        name = camera.name
        if name in self.cameras.keys():
            raise ValueError(f"Camera {name} already exists for robot {self.name}")
        self.cameras[name] = camera

    def add_camera(self, name: str, camera_link: str, minimal_height: float, maximal_height: float):
        """
        Creates and adds a CameraDescription object to the RobotDescription. Minimal and maximal height of the camera are
        relevant if the robot has a moveable torso or the camera is mounted on a moveable part of the robot. Otherwise
        both values can be the same.

        :param name: Name of the CameraDescription object
        :param camera_link: Link of the camera in the URDF
        :param minimal_height: Minimal height of the camera
        :param maximal_height: Maximal height of the camera
        :return:
        """
        camera_desc = CameraDescription(name, camera_link, minimal_height, maximal_height)
        self.cameras[name] = camera_desc

    def get_manipulator_chains(self) -> List[KinematicChainDescription]:
        """
        Returns a list of all manipulator chains of the robot which posses an end effector.

        :return: A list of KinematicChainDescription objects
        """
        result = []
        for chain in self.kinematic_chains.values():
            if chain.end_effector:
                result.append(chain)
        return result

    def get_camera_frame(self) -> str:
        """
        Quick method to get the name of a link of a camera. Uses the first camera in the list of cameras.

        :return: A name of the link of a camera
        """
        return self.cameras[list(self.cameras.keys())[0]].link_name


class KinematicChainDescription:
    """
    Represents a kinematic chain of a robot. A Kinematic chain is a chain of links and joints that are connected to each
    other and can be moved.

    This class contains all necessary information about the chain, like the start and end
    link, the URDF object and the joints of the chain.
    """
    def __init__(self, name: str, start_link: str, end_link: str, urdf_object: URDF, include_fixed_joints=False):
        """
        Initialize the KinematicChainDescription object.

        :param name: Name of the chain
        :param start_link: First link of the chain
        :param end_link: Last link of the chain
        :param urdf_object: URDF object of the robot which is used to get the chain
        :param include_fixed_joints: If True, fixed joints are included in the chain
        """
        self.name = name
        self.start_link = start_link
        self.end_link = end_link
        self.urdf_object = urdf_object
        self.include_fixed_joints = include_fixed_joints
        self.link_names = []
        self.joint_names = []
        self.end_effector = None
        self.static_joint_states = {}

        self._init_links()
        self._init_joints()

    def _init_links(self):
        """
        Initializes the links of the chain by getting the chain from the URDF object.
        """
        self.link_names = self.urdf_object.get_chain(self.start_link, self.end_link, joints=False)

    def _init_joints(self):
        """
        Initializes the joints of the chain by getting the chain from the URDF object.
        """
        self.joint_names = self.urdf_object.get_chain(self.start_link, self.end_link, links=False)

    def get_joints(self) -> List[str]:
        """
        Returns a list of all joints of the chain.

        :return: List of joint names
        """
        return self.joint_names

    def get_links(self) -> List[str]:
        """
        Returns a list of all links of the chain.

        :return: List of link names
        """
        return self.link_names

    @property
    def links(self) -> List[str]:
        """
        Property to get the links of the chain.

        :return: List of link names
        """
        return self.get_links()

    @property
    def joints(self) -> List[str]:
        """
        Property to get the joints of the chain.

        :return: List of joint names
        """
        return self.get_joints()

    def add_static_joint_states(self, name: str, states: dict):
        """
        Adds static joint states to the chain. These define a specific configuration of the chain.

        :param name: Name of the static joint states
        :param states: Dictionary of joint names and their values
        """
        for joint_name, value in states.items():
            if joint_name not in self.joint_names:
                raise ValueError(f"Joint {joint_name} is not part of the chain")
        self.static_joint_states[name] = states

    def get_static_joint_states(self, name: str) -> Dict[str, float]:
        """
        Returns the dictionary of static joint states for a given name of the static joint states.

        :param name: Name of the static joint states
        :return: Dictionary of joint names and their values
        """
        try:
            return self.static_joint_states[name]
        except KeyError:
            rospy.logerr(f"Static joint states for chain {name} not found")


class CameraDescription:
    """
    Represents a camera mounted on a robot. Contains all necessary information about the camera, like the link name,
    minimal and maximal height, horizontal and vertical angle and the front facing axis.
    """
    def __init__(self, name: str, link_name: str, minimal_height: float, maximal_height: float,
                 horizontal_angle: float = 20, vertical_angle: float = 20, front_facing_axis: List[float] = None):
        """
        Initialize the CameraDescription object.

        :param name: Name of the camera
        :param link_name: Name of the link in the URDF
        :param minimal_height: Minimal height the camera can be at
        :param maximal_height: Maximal height the camera can be at
        :param horizontal_angle: Horizontal opening angle of the camera
        :param vertical_angle: Vertical opening angle of the camera
        :param front_facing_axis: Axis along which the camera taking the image
        """
        self.name = name
        self.link_name = link_name
        self.minimal_height = minimal_height
        self.maximal_height = maximal_height
        self.horizontal_angle = horizontal_angle
        self.vertical_angle = vertical_angle
        self.front_facing_axis = front_facing_axis if front_facing_axis else [0, 0, 1]


class EndEffectorDescription:
    """
    Describes an end effector of robot. Contains all necessary information about the end effector, like the
    base link, the tool frame, the URDF object and the static joint states.
    """
    def __init__(self, name: str, start_link: str, tool_frame: str, urdf_object: URDF):
        """
        Initialize the EndEffectorDescription object.

        :param name: Name of the end effector
        :param start_link: Root link of the end effector, every link below this link in the URDF is part of the end effector
        :param tool_frame: Name of the tool frame link in the URDf
        :param urdf_object: URDF object of the robot
        """
        self.name = name
        self.start_link = start_link
        self.tool_frame = tool_frame
        self.urdf_object = urdf_object
        self.link_names = []
        self.joint_names = []
        self.static_joint_states = {}
        self._init_links_joints()

    def _init_links_joints(self):
        """
        Traverses the URDF object to get all links and joints of the end effector below the start link.1
        """
        start_link_obj = self.urdf_object.link_map[self.start_link]
        links = [start_link_obj.name]
        while len(links) != 0:
            link = links.pop()
            self.link_names.append(link)
            if link not in self.urdf_object.child_map:
                continue
            else:
                children = self.urdf_object.child_map[link]
            for joint, link in children:
                self.joint_names.append(joint)
                links.insert(0, link)

    def add_static_joint_states(self, name: str, states: dict):
        """
        Adds static joint states to the end effector. These define a specific configuration of the end effector. Like
        open and close configurations of a gripper.

        :param name: Name of the static joint states
        :param states: Dictionary of joint names and their values
        """
        for joint_name, value in states.items():
            if joint_name not in self.joint_names:
                raise ValueError(f"Joint {joint_name} is not part of the chain")
        self.static_joint_states[name] = states
