# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import rospy
from typing_extensions import List
from urdf_parser_py.urdf import URDF

class RobotDescriptionManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self.descriptions = {}
        self._initialized = True

    def load_description(self, name):
        if name in self.descriptions.keys():
            RobotDescription.current_robot_description = self.descriptions[name]
            return self.descriptions[name]
        else:
            rospy.logerr(f"Robot description {name} not found")


class RobotDescription:

    current_robot_description = None
    def __init__(self, name: str, base_link: str, torso_link: str, torso_joint: str, urdf_path: str):
        self.name = name
        self.base_link = base_link
        self.torso_link = torso_link
        self.torso_joint = torso_joint
        self.urdf_path = URDF.from_xml_file(urdf_path)
        self.kinematic_chains = {}
        self.cameras = []

    def add_kinematic_chain_description(self, chain: KinematicChainDescription):
        if chain.name in self.kinematic_chains.keys():
            raise ValueError(f"Chain {chain.name} already exists for robot {self.name}")
        self.kinematic_chains[chain.name] = chain

    def add_kinematic_chain(self, name: str, start_link: str, end_link: str):
        if name in self.kinematic_chains.keys():
            raise ValueError(f"Chain {name} already exists for robot {self.name}")
        chain = KinematicChainDescription(name, start_link, end_link, self.urdf_path)
        self.add_kinematic_chain_description(chain)

    def get_manipulator_chains(self) -> List[KinematicChainDescription]:
        result = []
        for chain in self.kinematic_chains:
            if chain.end_effector:
                result.append(chain)
        return result


class KinematicChainDescription:
    def __init__(self, name: str, start_link: str, end_link: str, urdf_object: URDF):
        self.name = name
        self.start_link = start_link
        self.end_link = end_link
        self.urdf_object = urdf_object
        self.link_names = []
        self.joint_names = []
        self.end_effector = None
        self.static_joint_states = {}

    def _init_links(self):
        self.link_names = self.urdf_object.get_chain(self.start_link, self.end_link, links=True)

    def _init_joints(self):
        self.joint_names = self.urdf_object.get_chain(self.start_link, self.end_link, joints=True)

    def get_joints(self) -> List[str]:
        return self.joint_names

    def get_links(self) -> List[str]:
        return self.link_names

    @property
    def links(self) -> List[str]:
        return self.get_links()

    @property
    def joints(self) -> List[str]:
        return self.get_joints()

    def add_static_joint_states(self, name: str, states: dict):
        for name, value in states.items():
            if name not in self.joint_names:
                raise ValueError(f"Joint {name} is not part of the chain")
        self.static_joint_states[name] = states

    def get_static_joint_states(self, name: str) -> dict:
        try:
            return self.static_joint_states[name]
        except KeyError:
            rospy.logerr(f"Static joint states for chain {name} not found")


class CameraDescription:
    def __init__(self, name: str, link_name: str, minimal_height: float, maximal_height: float,
                 horizontal_angle: float = 20, vertical_angle: float = 20, front_facing_axis: List[float] = None):
        self.name = name
        self.link_name = link_name
        self.minimal_height = minimal_height
        self.maximal_height = maximal_height
        self.horizontal_angle = horizontal_angle
        self.vertical_angle = vertical_angle
        self.front_facing_axis = front_facing_axis if front_facing_axis else [0, 0, 1]


class EndEffectorDescription:
    def __init__(self, name: str, start_link: str, tool_frame: str, urdf_object: URDF):
        self.name = name
        self.start_link = start_link
        self.tool_frame = tool_frame
        self.urdf_object = urdf_object
        self.link_names = []
        self.joint_names = []
        self.static_joint_states = {}
        self._init_links_joints()

    def _init_links_joints(self):
        start_link_obj = self.urdf_object.link_map[self.start_link]
        links = [start_link_obj.name]
        while len(len) != 0:
            link = links.pop()
            self.link_names.append(link)
            children = self.urdf_object.child_map[link]
            for joint, link in children:
                self.joint_names.append(joint.name)
                links.insert(0, link.name)
                # links.append(link.name)

    def add_static_joint_states(self, name: str, states: dict):
        for name, value in states.items():
            if name not in self.joint_names:
                raise ValueError(f"Joint {name} is not part of the chain")
        self.static_joint_states[name] = states
