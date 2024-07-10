# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import rospy
from typing_extensions import List, Dict, Union, Optional
from urdf_parser_py.urdf import URDF

from .utils import suppress_stdout_stderr
from .datastructures.enums import Arms, Grasp, GripperState, GripperType


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
        self.descriptions: Dict[str, RobotDescription] = {}
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
    current_robot_description: RobotDescription = None
    """
    The currently loaded robot description.
    """
    name: str
    """
    Name of the robot
    """
    base_link: str
    """
    Base link of the robot
    """
    torso_link: str
    """
    Torso link of the robot
    """
    torso_joint: str
    """
    Torso joint of the robot
    """
    urdf_object: URDF
    """
    Parsed URDF of the robot
    """
    kinematic_chains: Dict[str, KinematicChainDescription]
    """
    All kinematic chains defined for this robot
    """
    cameras: Dict[str, CameraDescription]
    """
    All cameras defined for this robot
    """
    grasps: Dict[Grasp, List[float]]
    """
    The orientations of the end effector for different grasps
    """
    links: List[str]
    """
    All links defined in the URDF
    """
    joints: List[str]
    """
    All joints defined in the URDF, by default fixed joints are not included
    """

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
            # Since parsing URDF causes a lot of warning messages which can't be deactivated, we suppress them
            self.urdf_object = URDF.from_xml_file(urdf_path)
        self.kinematic_chains: Dict[str, KinematicChainDescription] = {}
        self.cameras: Dict[str, CameraDescription] = {}
        self.grasps: Dict[Grasp, List[float]] = {}
        self.links: List[str] = [l.name for l in self.urdf_object.links]
        self.joints: List[str] = [j.name for j in self.urdf_object.joints]

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

    def add_grasp_orientation(self, grasp: Grasp, orientation: List[float]):
        """
        Adds a grasp orientation to the robot description. This is used to define the orientation of the end effector
        when grasping an object.

        :param grasp: Gasp from the Grasp enum which should be added
        :param orientation: List of floats representing the orientation
        """
        self.grasps[grasp] = orientation

    def add_grasp_orientations(self, orientations: Dict[Grasp, List[float]]):
        """
        Adds multiple grasp orientations to the robot description. This is used to define the orientation of the end effector
        when grasping an object.

        :param orientations: Dictionary of grasp orientations
        """
        self.grasps.update(orientations)

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

    def get_default_camera(self) -> CameraDescription:
        """
        Returns the first camera in the list of cameras.

        :return: A CameraDescription object
        """
        return self.cameras[list(self.cameras.keys())[0]]

    def get_static_joint_chain(self, kinematic_chain_name: str, configuration_name: str):
        """
        Returns the static joint states of a kinematic chain for a specific configuration. When trying to access one of
        the robot arms the function `:func: get_arm_chain` should be used.

        :param kinematic_chain_name:
        :param configuration_name:
        :return:
        """
        if kinematic_chain_name in self.kinematic_chains.keys():
            if configuration_name in self.kinematic_chains[kinematic_chain_name].static_joint_states.keys():
                return self.kinematic_chains[kinematic_chain_name].static_joint_states[configuration_name]
            else:
                raise ValueError(
                    f"There is no static joint state with the name {configuration_name} for Kinematic chain {kinematic_chain_name} of robot {self.name}")
        else:
            raise ValueError(f"There is no KinematicChain with name {kinematic_chain_name} for robot {self.name}")

    def get_parent(self, name: str) -> str:
        """
        Returns the parent of a link or joint in the URDF. Always returns the imeadiate parent, for a link this is a joint
        and vice versa.

        :param name: Name of the link or joint in the URDF
        :return: Name of the parent link or joint
        """
        if name not in self.links and name not in self.joints:
            raise ValueError(f"Link or joint {name} not found in URDF")
        if name in self.links:
            if name in self.urdf_object.parent_map:
                parent_joint, parent_link = self.urdf_object.parent_map[name]
                return parent_joint
            else:
                raise ValueError(f"Link {name} has no parent")
        elif name in self.joints:
            parent_link = self.urdf_object.joint_map[name].parent
            return parent_link

    def get_child(self, name: str, return_multiple_children: bool = False) -> Union[str, List[str]]:
        """
        Returns the child of a link or joint in the URDF. Always returns the immediate child, for a link this is a joint
        and vice versa. Since a link can have multiple children, the return_multiple_children parameter can be set to
        True to get a list of all children.

        :param name: Name of the link or joint in the URDF
        :param return_multiple_children: If True, a list of all children is returned
        :return: Name of the child link or joint or a list of all children
        """
        if name not in self.links and name not in self.joints:
            raise ValueError(f"Link or joint {name} not found in URDF")
        if name in self.links:
            if name in self.urdf_object.child_map:
                children = self.urdf_object.child_map[name]
                # A link can have multiple children
                child_joints = [child[0] for child in children]
                if return_multiple_children:
                    child_joints.reverse()
                    return child_joints
                else:
                    return child_joints[0]

            else:
                raise ValueError(f"Link {name} has no children")
        elif name in self.joints:
            child_link = self.urdf_object.joint_map[name].child
            return child_link

    def get_arm_chain(self, arm: Arms) -> Union[KinematicChainDescription, List[KinematicChainDescription]]:
        """
        Returns the kinematic chain of a specific arm. If the arm is set to BOTH, all kinematic chains are returned.

        :param arm: Arm for which the chain should be returned
        :return: KinematicChainDescription object of the arm
        """
        if arm == Arms.BOTH:
            return list(filter(lambda chain: chain.arm_type is not None, self.kinematic_chains.values()))
        for chain in self.kinematic_chains.values():
            if chain.arm_type == arm:
                return chain
        raise ValueError(f"There is no Kinematic Chain for the Arm {arm}")


class KinematicChainDescription:
    """
    Represents a kinematic chain of a robot. A Kinematic chain is a chain of links and joints that are connected to each
    other and can be moved.

    This class contains all necessary information about the chain, like the start and end
    link, the URDF object and the joints of the chain.
    """

    name: str
    """
    Name of the chain
    """
    start_link: str
    """
    First link of the chain
    """
    end_link: str
    """
    Last link of the chain
    """
    urdf_object: URDF
    """
    Parsed URDF of the robot
    """
    include_fixed_joints: bool
    """
    If True, fixed joints are included in the chain
    """
    link_names: List[str]
    """
    List of all links in the chain
    """
    joint_names: List[str]
    """
    List of all joints in the chain
    """
    end_effector: EndEffectorDescription
    """
    End effector of the chain, if there is one
    """
    arm_type: Arms
    """
    Type of the arm, if the chain is an arm
    """
    static_joint_states: Dict[str, Dict[str, float]]
    """
    Dictionary of static joint states for the chain
    """

    def __init__(self, name: str, start_link: str, end_link: str, urdf_object: URDF, arm_type: Arms = None,
                 include_fixed_joints=False):
        """
        Initialize the KinematicChainDescription object.

        :param name: Name of the chain
        :param start_link: First link of the chain
        :param end_link: Last link of the chain
        :param urdf_object: URDF object of the robot which is used to get the chain
        :param arm_type: Type of the arm, if the chain is an arm
        :param include_fixed_joints: If True, fixed joints are included in the chain
        """
        self.name: str = name
        self.start_link: str = start_link
        self.end_link: str = end_link
        self.urdf_object: URDF = urdf_object
        self.include_fixed_joints: bool = include_fixed_joints
        self.link_names: List[str] = []
        self.joint_names: List[str] = []
        self.arm_type: Arms = arm_type
        self.static_joint_states: Dict[str, Dict[str, float]] = {}
        self.end_effector = None

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
        joints = self.urdf_object.get_chain(self.start_link, self.end_link, links=False)
        self.joint_names = list(filter(lambda j: self.urdf_object.joint_map[j].type != "fixed" or self.include_fixed_joints, joints))

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

    def get_tool_frame(self) -> str:
        """
        Returns the name of the tool frame of the end effector of this chain, if it has an end effector.

        :return: The name of the link of the tool frame in the URDF.
        """
        if self.end_effector:
            return self.end_effector.tool_frame
        else:
            raise ValueError(f"The Kinematic chain {self.name} has no end-effector")

    def get_static_gripper_state(self, state: GripperState) -> Dict[str, float]:
        """
        Returns the static joint states for the gripper of the chain.

        :param state: Name of the static joint states
        :return: Dictionary of joint names and their values
        """
        if self.end_effector:
            return self.end_effector.static_joint_states[state]
        else:
            raise ValueError(f"The Kinematic chain {self.name} has no end-effector")


class CameraDescription:
    """
    Represents a camera mounted on a robot. Contains all necessary information about the camera, like the link name,
    minimal and maximal height, horizontal and vertical angle and the front facing axis.
    """
    name: str
    """
    Name of the camera
    """
    link_name: str
    """
    Name of the link in the URDF
    """
    minimal_height: float
    """
    Minimal height the camera can be at
    """
    maximal_height: float
    """
    Maximal height the camera can be at
    """
    horizontal_angle: float
    """
    Horizontal opening angle of the camera
    """
    vertical_angle: float
    """
    Vertical opening angle of the camera
    """
    front_facing_axis: List[int]
    """
    Axis along which the camera is taking the image
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
        self.name: str = name
        self.link_name: str = link_name
        self.minimal_height: float = minimal_height
        self.maximal_height: float = maximal_height
        self.horizontal_angle: float = horizontal_angle
        self.vertical_angle: float = vertical_angle
        self.front_facing_axis: List[int] = front_facing_axis if front_facing_axis else [0, 0, 1]


class EndEffectorDescription:
    """
    Describes an end effector of robot. Contains all necessary information about the end effector, like the
    base link, the tool frame, the URDF object and the static joint states.
    """
    name: str
    """
    Name of the end effector
    """
    start_link: str
    """
    Root link of the end effector, every link below this link in the URDF is part of the end effector
    """
    tool_frame: str
    """
    Name of the tool frame link in the URDf
    """
    urdf_object: URDF
    """
    Parsed URDF of the robot
    """
    link_names: List[str]
    """
    List of all links in the end effector
    """
    joint_names: List[str]
    """
    List of all joints in the end effector
    """
    static_joint_states: Dict[GripperState, Dict[str, float]]
    """
    Dictionary of static joint states for the end effector
    """
    end_effector_type: GripperType
    """
    Type of the gripper
    """
    opening_distance: float
    """
    Distance the gripper can open, in cm
    """

    def __init__(self, name: str, start_link: str, tool_frame: str, urdf_object: URDF):
        """
        Initialize the EndEffectorDescription object.

        :param name: Name of the end effector
        :param start_link: Root link of the end effector, every link below this link in the URDF is part of the end effector
        :param tool_frame: Name of the tool frame link in the URDf
        :param urdf_object: URDF object of the robot
        """
        self.name: str = name
        self.start_link: str = start_link
        self.tool_frame: str = tool_frame
        self.urdf_object: URDF = urdf_object
        self.link_names: List[str] = []
        self.joint_names: List[str] = []
        self.static_joint_states: Dict[GripperState, Dict[str, float]] = {}
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

    def add_static_joint_states(self, name: GripperState, states: dict):
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

    @property
    def links(self) -> List[str]:
        """
        Property to get the links of the chain.

        :return: List of link names
        """
        return self.link_names

    @property
    def joints(self) -> List[str]:
        """
        Property to get the joints of the chain.

        :return: List of joint names
        """
        return self.joint_names
