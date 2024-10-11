import os
import pathlib
import xml.etree.ElementTree as ET

import numpy as np

from ..ros.logging import logerr
from ..ros.ros_tools import create_ros_pack, ResourceNotFound, get_parameter
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from typing_extensions import Union, List, Optional, Dict, Tuple
from urdf_parser_py import urdf
from urdf_parser_py.urdf import (URDF, Collision, Box as URDF_Box, Cylinder as URDF_Cylinder,
                                 Sphere as URDF_Sphere, Mesh as URDF_Mesh)

from ..datastructures.enums import JointType
from ..datastructures.pose import Pose
from ..description import JointDescription as AbstractJointDescription, \
    LinkDescription as AbstractLinkDescription, ObjectDescription as AbstractObjectDescription
from ..datastructures.dataclasses import Color, VisualShape, BoxVisualShape, CylinderVisualShape, \
    SphereVisualShape, MeshVisualShape
from ..failures import MultiplePossibleTipLinks
from ..utils import suppress_stdout_stderr


class LinkDescription(AbstractLinkDescription):
    """
    A class that represents a link description of an object.
    """

    def __init__(self, urdf_description: urdf.Link):
        super().__init__(urdf_description)

    @property
    def geometry(self) -> Union[VisualShape, None]:
        """
        :return: The geometry type of the URDF collision element of this link.
        """
        if self.collision is None:
            return None
        urdf_geometry = self.collision.geometry
        return self._get_visual_shape(urdf_geometry)

    @staticmethod
    def _get_visual_shape(urdf_geometry) -> Union[VisualShape, None]:
        """
        :param urdf_geometry: The URDFGeometry for which the visual shape is returned.
        :return: the VisualShape of the given URDF geometry.
        """
        if isinstance(urdf_geometry, URDF_Box):
            half_extents = np.array(urdf_geometry.size) / 2
            return BoxVisualShape(Color(), [0, 0, 0], half_extents.tolist())
        if isinstance(urdf_geometry, URDF_Cylinder):
            return CylinderVisualShape(Color(), [0, 0, 0], urdf_geometry.radius, urdf_geometry.length)
        if isinstance(urdf_geometry, URDF_Sphere):
            return SphereVisualShape(Color(), [0, 0, 0], urdf_geometry.radius)
        if isinstance(urdf_geometry, URDF_Mesh):
            return MeshVisualShape(Color(), [0, 0, 0], urdf_geometry.scale, urdf_geometry.filename)
        return None

    @property
    def origin(self) -> Union[Pose, None]:
        if self.collision is None:
            return None
        if self.collision.origin is None:
            return None
        return Pose(self.collision.origin.xyz,
                    quaternion_from_euler(*self.collision.origin.rpy).tolist())

    @property
    def name(self) -> str:
        return self.parsed_description.name

    @property
    def collision(self) -> Collision:
        return self.parsed_description.collision


class JointDescription(AbstractJointDescription):
    urdf_type_map = {'unknown': JointType.UNKNOWN,
                     'revolute': JointType.REVOLUTE,
                     'continuous': JointType.CONTINUOUS,
                     'prismatic': JointType.PRISMATIC,
                     'floating': JointType.FLOATING,
                     'planar': JointType.PLANAR,
                     'fixed': JointType.FIXED}

    pycram_type_map = {pycram_type: urdf_type for urdf_type, pycram_type in urdf_type_map.items()}

    def __init__(self, urdf_description: urdf.Joint, is_virtual: Optional[bool] = False):
        super().__init__(urdf_description, is_virtual=is_virtual)

    @property
    def origin(self) -> Pose:
        return Pose(self.parsed_description.origin.xyz,
                    quaternion_from_euler(*self.parsed_description.origin.rpy))

    @property
    def name(self) -> str:
        return self.parsed_description.name

    @property
    def has_limits(self) -> bool:
        return bool(self.parsed_description.limit)

    @property
    def type(self) -> JointType:
        """
        :return: The type of this joint.
        """
        return self.urdf_type_map[self.parsed_description.type]

    @property
    def axis(self) -> Point:
        """
        :return: The axis of this joint, for example the rotation axis for a revolute joint.
        """
        return Point(*self.parsed_description.axis)

    @property
    def lower_limit(self) -> Union[float, None]:
        """
        :return: The lower limit of this joint, or None if the joint has no limits.
        """
        if self.has_limits:
            return self.parsed_description.limit.lower
        else:
            return None

    @property
    def upper_limit(self) -> Union[float, None]:
        """
        :return: The upper limit of this joint, or None if the joint has no limits.
        """
        if self.has_limits:
            return self.parsed_description.limit.upper
        else:
            return None

    @property
    def parent(self) -> str:
        """
        :return: The name of the parent link of this joint.
        """
        return self.parsed_description.parent

    @property
    def child(self) -> str:
        """
        :return: The name of the child link of this joint.
        """
        return self.parsed_description.child

    @property
    def damping(self) -> float:
        """
        :return: The damping of this joint.
        """
        return self.parsed_description.dynamics.damping

    @property
    def friction(self) -> float:
        """
        :return: The friction of this joint.
        """
        return self.parsed_description.dynamics.friction


class ObjectDescription(AbstractObjectDescription):
    """
    A class that represents an object description of an object.
    """

    class Link(AbstractObjectDescription.Link, LinkDescription):
        ...

    class RootLink(AbstractObjectDescription.RootLink, Link):
        ...

    class Joint(AbstractObjectDescription.Joint, JointDescription):
        ...

    @property
    def child_map(self) -> Dict[str, List[Tuple[str, str]]]:
        """
        :return: A dictionary mapping the name of a link to its children which are represented as a tuple of the child
            joint name and the link name.
        """
        return self.parsed_description.child_map

    @property
    def parent_map(self) -> Dict[str, Tuple[str, str]]:
        """
        :return: A dictionary mapping the name of a link to its parent joint and link as a tuple.
        """
        return self.parsed_description.parent_map

    @property
    def link_map(self) -> Dict[str, LinkDescription]:
        """
        :return: A dictionary mapping the name of a link to its description.
        """
        if self._link_map is None:
            self._link_map = {link.name: link for link in self.links}
        return self._link_map

    @property
    def joint_map(self) -> Dict[str, JointDescription]:
        """
        :return: A dictionary mapping the name of a joint to its description.
        """
        if self._joint_map is None:
            self._joint_map = {joint.name: joint for joint in self.joints}
        return self._joint_map

    def add_joint(self, name: str, child: str, joint_type: JointType,
                  axis: Point, parent: Optional[str] = None, origin: Optional[Pose] = None,
                  lower_limit: Optional[float] = None, upper_limit: Optional[float] = None,
                  is_virtual: Optional[bool] = False) -> None:
        """
        Add a joint to the object description, could be a virtual joint as well.
        For documentation of the parameters, see :meth:`pycram.description.ObjectDescription.add_joint`.
        """
        if lower_limit is not None or upper_limit is not None:
            limit = urdf.JointLimit(lower=lower_limit, upper=upper_limit)
        else:
            limit = None
        if origin is not None:
            origin = urdf.Pose(origin.position_as_list(), euler_from_quaternion(origin.orientation_as_list()))
        if axis is not None:
            axis = [axis.x, axis.y, axis.z]
        if parent is None:
            parent = self.get_root()
        else:
            parent = self.get_link_by_name(parent).parsed_description
        joint = urdf.Joint(name,
                           parent,
                           self.get_link_by_name(child).parsed_description,
                           JointDescription.pycram_type_map[joint_type],
                           axis, origin, limit)
        self.parsed_description.add_joint(joint)
        if is_virtual:
            self.virtual_joint_names.append(name)

    def load_description(self, path) -> URDF:
        with open(path, 'r') as file:
            # Since parsing URDF causes a lot of warning messages which can't be deactivated, we suppress them
            with suppress_stdout_stderr():
                return URDF.from_xml_string(file.read())

    def generate_from_mesh_file(self, path: str, name: str, save_path: str, color: Optional[Color] = Color()) -> None:
        """
        Generate a URDf file with the given .obj or .stl file as mesh. In addition, use the given rgba_color to create a
         material tag in the URDF. The URDF file will be saved to the given save_path.

        :param path: The path to the mesh file.
        :param name: The name of the object.
        :param save_path: The path to save the URDF file to.
        :param color: The color of the object.
        """
        urdf_template = '<?xml version="0.0" ?> \n \
                        <robot name="~a_object"> \n \
                         <link name="~a_main"> \n \
                            <visual> \n \
                                <geometry>\n \
                                    <mesh filename="~b" scale="1 1 1"/> \n \
                                </geometry>\n \
                                <material name="white">\n \
                                    <rgba_color rgba="~c"/>\n \
                                </material>\n \
                          </visual> \n \
                        <collision> \n \
                        <geometry>\n \
                            <mesh filename="~b" scale="1 1 1"/>\n \
                        </geometry>\n \
                        </collision>\n \
                        </link> \n \
                        </robot>'
        urdf_template = self.fix_missing_inertial(urdf_template)
        rgb = " ".join(list(map(str, color.get_rgba())))
        pathlib_obj = pathlib.Path(path)
        path = str(pathlib_obj.resolve())
        content = urdf_template.replace("~a", name).replace("~b", path).replace("~c", rgb)
        self.write_description_to_file(content, save_path)

    def generate_from_description_file(self, path: str, save_path: str, make_mesh_paths_absolute: bool = True) -> None:
        with open(path, mode="r") as f:
            urdf_string = self.fix_missing_inertial(f.read())
        urdf_string = self.remove_error_tags(urdf_string)
        urdf_string = self.fix_link_attributes(urdf_string)
        try:
            urdf_string = self.replace_relative_references_with_absolute_paths(urdf_string)
            urdf_string = self.fix_missing_inertial(urdf_string)
        except ResourceNotFound as e:
            logerr(f"Could not find resource package linked in this URDF")
            raise e
        urdf_string = self.make_mesh_paths_absolute(urdf_string, path) if make_mesh_paths_absolute else urdf_string
        self.write_description_to_file(urdf_string, save_path)

    def generate_from_parameter_server(self, name: str, save_path: str) -> None:
        urdf_string = get_parameter(name)
        urdf_string = self.replace_relative_references_with_absolute_paths(urdf_string)
        urdf_string = self.fix_missing_inertial(urdf_string)
        self.write_description_to_file(urdf_string, save_path)

    @property
    def joints(self) -> List[JointDescription]:
        """
        :return: A list of joints descriptions of this object.
        """
        if self._joints is None:
            self._joints = [JointDescription(joint) for joint in self.parsed_description.joints]
        return self._joints

    @property
    def links(self) -> List[LinkDescription]:
        """
        :return: A list of link descriptions of this object.
        """
        if self._links is None:
            self._links = [LinkDescription(link) for link in self.parsed_description.links]
        return self._links

    def get_root(self) -> str:
        """
        :return: the name of the root link of this object.
        """
        return self.parsed_description.get_root()

    def get_tip(self) -> str:
        """
        :return: the name of the tip link of this object.
        :raises MultiplePossibleTipLinks: If there are multiple possible tip links.
        """
        link = self.get_root()
        while link in self.parsed_description.child_map:
            children = self.parsed_description.child_map[link]
            if len(children) > 1:
                # Multiple children, can't decide which one to take (e.g. fingers of a hand)
                raise MultiplePossibleTipLinks(self.parsed_description.name, link, [child[1] for child in children])
            else:
                child = children[0][1]
                link = child
        return link

    def get_chain(self, start_link_name: str, end_link_name: str, joints: Optional[bool] = True,
                  links: Optional[bool] = True, fixed: Optional[bool] = True) -> List[str]:
        """
        :param start_link_name: The name of the start link of the chain.
        :param end_link_name: The name of the end link of the chain.
        :param joints: Whether to include joints in the chain.
        :param links: Whether to include links in the chain.
        :param fixed: Whether to include fixed joints in the chain.
        :return: the chain of links from 'start_link_name' to 'end_link_name'.
        """
        return self.parsed_description.get_chain(start_link_name, end_link_name, joints, links, fixed)

    @staticmethod
    def replace_relative_references_with_absolute_paths(urdf_string: str) -> str:
        """
        Change paths for files in the URDF from ROS paths and file dir references to paths in the file system. Since
        World (PyBullet legacy) can't deal with ROS package paths.

        :param urdf_string: The name of the URDf on the parameter server
        :return: The URDF string with paths in the filesystem instead of ROS packages
        """
        r = create_ros_pack()
        new_urdf_string = ""
        for line in urdf_string.split('\n'):
            if "package://" in line:
                s = line.split('//')
                s1 = s[1].split('/')
                path = r.get_path(s1[0])
                line = line.replace("package://" + s1[0], path)
            if 'file://' in line:
                line = line.replace("file://", './')
            new_urdf_string += line + '\n'

        return new_urdf_string

    @staticmethod
    def make_mesh_paths_absolute(urdf_string: str, urdf_file_path: str) -> str:
        """
        Convert all relative mesh paths in the URDF to absolute paths.

        :param urdf_string: The URDF description as string
        :param urdf_file_path: The path to the URDF file
        :returns: The new URDF description as string.
        """
        # Parse the URDF file
        root = ET.fromstring(urdf_string)

        # Iterate through all mesh tags
        for mesh in root.findall('.//mesh'):
            filename = mesh.attrib.get('filename', '')
            if filename:
                # If the filename is a relative path, convert it to an absolute path
                if not os.path.isabs(filename):
                    # Deduce the base path from the relative path
                    base_path = os.path.dirname(
                        os.path.abspath(os.path.join(os.path.dirname(urdf_file_path), filename)))
                    abs_path = os.path.abspath(os.path.join(base_path, os.path.basename(filename)))
                    mesh.set('filename', abs_path)

        return ET.tostring(root, encoding='unicode')

    @staticmethod
    def fix_missing_inertial(urdf_string: str) -> str:
        """
        Insert inertial tags for every URDF link that has no inertia.
        This is used to prevent Legacy(PyBullet) from dumping warnings in the terminal

        :param urdf_string: The URDF description as string
        :returns: The new, corrected URDF description as string.
        """

        inertia_tree = ET.ElementTree(ET.Element("inertial"))
        inertia_tree.getroot().append(ET.Element("mass", {"value": "0.1"}))
        inertia_tree.getroot().append(ET.Element("origin", {"rpy": "0 0 0", "xyz": "0 0 0"}))
        inertia_tree.getroot().append(ET.Element("inertia", {"ixx": "0.01",
                                                                      "ixy": "0",
                                                                      "ixz": "0",
                                                                      "iyy": "0.01",
                                                                      "iyz": "0",
                                                                      "izz": "0.01"}))

        # create tree from string
        tree = ET.ElementTree(ET.fromstring(urdf_string))

        for link_element in tree.iter("link"):
            inertial = [*link_element.iter("inertial")]
            if len(inertial) == 0:
                link_element.append(inertia_tree.getroot())

        return ET.tostring(tree.getroot(), encoding='unicode')

    @staticmethod
    def remove_error_tags(urdf_string: str) -> str:
        """
        Remove all tags in the removing_tags list from the URDF since these tags are known to cause errors with the
        URDF_parser

        :param urdf_string: String of the URDF from which the tags should be removed
        :return: The URDF string with the tags removed
        """
        tree = ET.ElementTree(ET.fromstring(urdf_string))
        removing_tags = ["gazebo", "transmission"]
        for tag_name in removing_tags:
            all_tags = tree.findall(tag_name)
            for tag in all_tags:
                tree.getroot().remove(tag)

        return ET.tostring(tree.getroot(), encoding='unicode')

    @staticmethod
    def fix_link_attributes(urdf_string: str) -> str:
        """
        Remove the attribute 'type' from links since this is not parsable by the URDF parser.

        :param urdf_string: The string of the URDF from which the attributes should be removed
        :return: The URDF string with the attributes removed
        """
        tree = ET.ElementTree(ET.fromstring(urdf_string))

        for link in tree.iter("link"):
            if "type" in link.attrib.keys():
                del link.attrib["type"]

        return ET.tostring(tree.getroot(), encoding='unicode')

    @staticmethod
    def get_file_extension() -> str:
        """
        :return: The file extension of the URDF file.
        """
        return '.urdf'

    @property
    def origin(self) -> Pose:
        return Pose(self.parsed_description.origin.xyz,
                    quaternion_from_euler(*self.parsed_description.origin.rpy))

    @property
    def name(self) -> str:
        return self.parsed_description.name
