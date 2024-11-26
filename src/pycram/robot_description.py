# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from enum import Enum

import numpy as np
import rospy
from typing_extensions import List, Dict, Union, Optional, Tuple
from urdf_parser_py.urdf import URDF

from .utils import load_urdf_silently, suppress_stdout_stderr
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
    links: List[str]
    """
    All links defined in the URDF
    """
    joints: List[str]
    """
    All joints defined in the URDF, by default fixed joints are not included
    """
    neck: Dict[str, List[str]]
    """
    Dictionary of neck links and joints
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
            self.urdf_object = load_urdf_silently(urdf_path)
        self.kinematic_chains: Dict[str, KinematicChainDescription] = {}
        self.cameras: Dict[str, CameraDescription] = {}
        self.links: List[str] = [l.name for l in self.urdf_object.links]
        self.joints: List[str] = [j.name for j in self.urdf_object.joints]
        self.costmap_offset: float = 0.3
        self.max_reach = None
        self.palm_axis = [0, 0, 1]
        self.distance_palm_to_tool_frame_left = None
        self.distance_palm_to_tool_frame_right = None

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
        return None
        # I think its nicer to return None if the arm is not found, so we can handle robots with only one arm better
        # raise ValueError(f"There is no Kinematic Chain for the Arm {arm}")

    def get_offset(self, name):
        """
        Returns the offset of a Joint in the URDF.
        :param name: The name of the Joint for which the offset will be returned.
        :return: The offset of the Joint
        """
        if name not in self.urdf_object.joint_map.keys():
            rospy.logerr(f"The name: {name} is not part of this robot URDF")
            return None

        offset = self.urdf_object.joint_map[name].origin
        return offset if offset else None

    def set_costmap_offset(self, offset: float):
        """
        Sets the costmap offset for the robot. This is used to define the distance between the robot and the costmap.
        :param offset: The offset in meters
        """
        self.costmap_offset = offset

    def get_costmap_offset(self) -> float:
        """
        Returns the costmap offset for the robot. This is used to define the distance between the robot and the costmap.
        :return: The offset in meters
        """
        return self.costmap_offset

    def get_torso_joint(self) -> str:
        """
        Returns the name of the torso joint of the robot.
        :return: The name of the torso joint
        """
        return self.torso_joint

    def get_tool_frame_joint(self, arm: Arms) -> str:
        """
        Retrieves the name of the tool frame joint for the specified arm.

        Args:
            arm (Arms): The arm for which to retrieve the tool frame joint.

        Returns:
            str: The name of the tool frame joint.
        """
        chain = self.get_arm_chain(arm)
        tool_frame = chain.end_effector.tool_frame
        return self.get_parent(tool_frame)

    def set_distance_palm_to_tool_frame(self, arm: Arms, distance: float):
        """
        Sets the distance between the palm and the tool frame for the specified arm.

        Args:
            arm (Arms): The arm for which to set the distance.
            distance (float): The distance to set between the palm and the tool frame.
        """
        if arm == Arms.LEFT:
            self.distance_palm_to_tool_frame_left = distance
        elif arm == Arms.RIGHT:
            self.distance_palm_to_tool_frame_right = distance

    def get_distance_palm_to_tool_frame(self, arm: Arms) -> float:
        """
        Retrieves the distance between the palm and the tool frame for the specified arm.

        If the distance is not yet set, it calculates the distance based on the robot's
        arm chain configuration and caches the result for future use.

        Args:
            arm (Arms): The arm for which to retrieve the palm-to-tool-frame distance.

        Returns:
            float: The distance between the palm and the tool frame.
        """
        if arm == Arms.LEFT:
            if not self.distance_palm_to_tool_frame_left:
                chain = self.get_arm_chain(arm)
                palm_link = chain.end_effector.start_link
                tool_frame = chain.end_effector.tool_frame
                distance = self.get_distance_between_links(palm_link, tool_frame)
                return distance
            else:
                return self.distance_palm_to_tool_frame_left

        if arm == Arms.RIGHT:
            if not self.distance_palm_to_tool_frame_right:
                chain = self.get_arm_chain(arm)
                palm_link = chain.end_effector.start_link
                tool_frame = chain.end_effector.tool_frame
                distance = self.get_distance_between_links(palm_link, tool_frame)
                return distance
            else:
                return self.distance_palm_to_tool_frame_right

    def get_distance_between_links(self, link1: str, link2: str) -> float:
        """
        Calculates the distance between two links in the URDF by summing the lengths
        of all intermediate links in the kinematic chain between them.

        Args:
            link1 (str): The name of the first link.
            link2 (str): The name of the second link.

        Returns:
            float: The total distance between the two links.
        """
        robot = self.urdf_object
        distance = 0.0

        kinematic_chain = robot.get_chain(link1, link2, joints=False)
        joints = robot.get_chain(link1, link2, links=False)

        for joint in joints:
            if self.get_parent(joint) in kinematic_chain and self.get_child(joint) in kinematic_chain:
                translation = robot.joint_map[joint].origin.position
                link_length = np.linalg.norm(translation)
                distance += link_length

        return distance

    def set_max_reach(self, start_link: str, end_link: str, factor: float = 0.65):
        """
        Calculates and sets the maximum reach of the robot by summing the lengths of
        links in the kinematic chain between the specified start and end links. The
        calculated reach is scaled by a given factor.

        Args:
            start_link (str): The starting link in the kinematic chain.
            end_link (str): The ending link in the kinematic chain.
            factor (float): A scaling factor to adjust the calculated maximum reach.
                            Defaults to 0.65.
        """
        robot = self.urdf_object
        max_reach = 0.0

        kinematic_chain = robot.get_chain(start_link, end_link, joints=False)
        joints = robot.get_chain(start_link, end_link, links=False)

        for joint in joints:
            if self.get_parent(joint) in kinematic_chain and self.get_child(
                    joint) in kinematic_chain:
                translation = robot.joint_map[joint].origin.position
                link_length = np.linalg.norm(translation)
                max_reach += link_length

        self.max_reach = max_reach * factor

    def get_max_reach(self) -> float:
        """
        Retrieves the maximum reach of the robot. If it has not been set, calculates
        the reach based on the primary manipulator chain and sets it.

        Returns:
            float: The maximum reach of the robot.
        """
        if not self.max_reach:
            manip = self.get_manipulator_chains()[0]
            self.set_max_reach(manip.start_link, manip.end_effector.tool_frame)

        return self.max_reach

    def get_joint_limits(self, joint_name: str) -> Optional[Tuple[float, float]]:
        """
        Retrieves the joint limits for a specified joint in the URDF.

        If the joint is not found or does not have defined limits, logs an error
        and returns `None`.

        Args:
            joint_name (str): The name of the joint for which to retrieve limits.

        Returns:
            Optional[Tuple[float, float]]: A tuple containing the lower and upper joint limits,
                                           or `None` if the joint is not found or has no limits.
        """
        if joint_name not in self.urdf_object.joint_map:
            rospy.logerr(f"Joint {joint_name} not found in URDF")
            return None
        joint = self.urdf_object.joint_map[joint_name]
        if joint.limit:
            return joint.limit.lower, joint.limit.upper
        else:
            return None

    def set_neck(self, yaw_joint: Optional[str] = None, pitch_joint: Optional[str] = None,
                 roll_joint: Optional[str] = None):
        """
        Defines the neck configuration of the robot by setting the yaw, pitch, and roll
        joints along with their corresponding links.

        Args:
            yaw_joint (Optional[str]): The name of the yaw joint. Defaults to None.
            pitch_joint (Optional[str]): The name of the pitch joint. Defaults to None.
            roll_joint (Optional[str]): The name of the roll joint. Defaults to None.
        """
        yaw_link = self.get_child(yaw_joint) if yaw_joint else None
        pitch_link = self.get_child(pitch_joint) if pitch_joint else None
        roll_link = self.get_child(roll_joint) if roll_joint else None
        self.neck = {
            "yaw": [yaw_link, yaw_joint],
            "pitch": [pitch_link, pitch_joint],
            "roll": [roll_link, roll_joint]
        }

    def get_neck(self) -> Dict[str, List[Optional[str]]]:
        """
        Retrieves the neck configuration of the robot, including links and joints for yaw,
        pitch, and roll.

        Returns:
            Dict[str, List[Optional[str]]]: A dictionary containing neck links and joints.
        """
        return self.neck

    def set_palm_axis(self, axis: List[float]):
        """
        Sets the direction axis for the robot's palm.

        Args:
            axis (List[float]): A list representing the direction of the palm axis.
        """
        self.palm_axis = axis

    def get_palm_axis(self) -> List[float]:
        """
        Retrieves the direction axis of the robot's palm.

        Returns:
            List[float]: The current direction of the palm axis.
        """
        return self.palm_axis


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
        self.joint_names = list(
            filter(lambda j: self.urdf_object.joint_map[j].type != "fixed" or self.include_fixed_joints, joints))

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

    def add_static_joint_states(self, name: Union[str, Enum], states: dict):
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
    grasps: Dict[Grasp, List[float]]
    """
    The orientations of the end effector for different grasps
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
        self.grasps: Dict[Grasp, List[float]] = {}
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

    def generate_all_grasp_orientations_from_front_grasp(self, front_orientation: List[float]):
        """
        Generates all grasp orientations based on a given front-facing orientation.

        This method calculates orientations for six grasp directions (front, back, left, right,
        top, and bottom) relative to a specified front-facing orientation. Each orientation
        is computed by applying a quaternion multiplication between the front orientation and
        predefined relative rotations.

        Args:
            front_orientation (List[float]): A quaternion representing the front-facing orientation
                                             as [x, y, z, w].
        """
        relative_rotations = {
            Grasp.FRONT: [0, 0, 0, 1],
            Grasp.BACK: [0, 0, 1, 0],
            Grasp.LEFT: [0, 0, -0.707, 0.707],
            Grasp.RIGHT: [0, 0, 0.707, 0.707],
            Grasp.TOP: [0, 0.707, 0, 0.707],
            Grasp.BOTTOM: [0, -0.707, 0, 0.707]
        }

        all_orientations = {}

        for grasp, relative_rotation in relative_rotations.items():
            x1, y1, z1, w1 = front_orientation
            x2, y2, z2, w2 = relative_rotation

            grasp_orientation_x = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
            grasp_orientation_y = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
            grasp_orientation_z = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1
            grasp_orientation_w = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1

            all_orientations[grasp] = [grasp_orientation_x, grasp_orientation_y,
                                       grasp_orientation_z, grasp_orientation_w]

        self.grasps = all_orientations

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
