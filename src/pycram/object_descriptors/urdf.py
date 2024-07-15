import pathlib
from xml.etree import ElementTree

import rospkg
import rospy
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from typing_extensions import Union, List, Optional
from urdf_parser_py import urdf
from urdf_parser_py.urdf import (URDF, Collision, Box as URDF_Box, Cylinder as URDF_Cylinder,
                                 Sphere as URDF_Sphere, Mesh as URDF_Mesh)

from ..datastructures.enums import JointType
from ..datastructures.pose import Pose
from ..description import JointDescription as AbstractJointDescription, \
    LinkDescription as AbstractLinkDescription, ObjectDescription as AbstractObjectDescription
from ..datastructures.dataclasses import Color, VisualShape, BoxVisualShape, CylinderVisualShape, \
    SphereVisualShape, MeshVisualShape
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
        Returns the geometry type of the URDF collision element of this link.
        """
        if self.collision is None:
            return None
        urdf_geometry = self.collision.geometry
        return self._get_visual_shape(urdf_geometry)

    @staticmethod
    def _get_visual_shape(urdf_geometry) -> Union[VisualShape, None]:
        """
        Returns the VisualShape of the given URDF geometry.
        """
        if isinstance(urdf_geometry, URDF_Box):
            return BoxVisualShape(Color(), [0, 0, 0], urdf_geometry.size)
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

    def __init__(self, urdf_description: urdf.Joint):
        super().__init__(urdf_description)

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
    def parent_link_name(self) -> str:
        """
        :return: The name of the parent link of this joint.
        """
        return self.parsed_description.parent

    @property
    def child_link_name(self) -> str:
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

    def load_description(self, path) -> URDF:
        with open(path, 'r') as file:
            # Since parsing URDF causes a lot of warning messages which can't be deactivated, we suppress them
            with suppress_stdout_stderr():
                return URDF.from_xml_string(file.read())

    def generate_from_mesh_file(self, path: str, name: str, color: Optional[Color] = Color()) -> str:
        """
        Generates an URDf file with the given .obj or .stl file as mesh. In addition, the given rgba_color will be
        used to create a material tag in the URDF.

        :param path: The path to the mesh file.
        :param name: The name of the object.
        :param color: The color of the object.
        :return: The absolute path of the created file
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
        return content

    def generate_from_description_file(self, path: str) -> str:
        with open(path, mode="r") as f:
            urdf_string = self.fix_missing_inertial(f.read())
            urdf_string = self.remove_error_tags(urdf_string)
            urdf_string = self.fix_link_attributes(urdf_string)
            try:
                urdf_string = self.correct_urdf_string(urdf_string)
            except rospkg.ResourceNotFound as e:
                rospy.logerr(f"Could not find resource package linked in this URDF")
                raise e
        return urdf_string

    def generate_from_parameter_server(self, name: str) -> str:
        urdf_string = rospy.get_param(name)
        return self.correct_urdf_string(urdf_string)

    def get_link_by_name(self, link_name: str) -> LinkDescription:
        """
        :return: The link description with the given name.
        """
        for link in self.links:
            if link.name == link_name:
                return link
        raise ValueError(f"Link with name {link_name} not found")

    @property
    def links(self) -> List[LinkDescription]:
        """
        :return: A list of links descriptions of this object.
        """
        return [LinkDescription(link) for link in self.parsed_description.links]

    def get_joint_by_name(self, joint_name: str) -> JointDescription:
        """
        :return: The joint description with the given name.
        """
        for joint in self.joints:
            if joint.name == joint_name:
                return joint
        raise ValueError(f"Joint with name {joint_name} not found")

    @property
    def joints(self) -> List[JointDescription]:
        """
        :return: A list of joints descriptions of this object.
        """
        return [JointDescription(joint) for joint in self.parsed_description.joints]

    def get_root(self) -> str:
        """
        :return: the name of the root link of this object.
        """
        return self.parsed_description.get_root()

    def get_chain(self, start_link_name: str, end_link_name: str) -> List[str]:
        """
        :return: the chain of links from 'start_link_name' to 'end_link_name'.
        """
        return self.parsed_description.get_chain(start_link_name, end_link_name)

    def correct_urdf_string(self, urdf_string: str) -> str:
        """
        Changes paths for files in the URDF from ROS paths to paths in the file system. Since World (PyBullet legacy)
        can't deal with ROS package paths.

        :param urdf_string: The name of the URDf on the parameter server
        :return: The URDF string with paths in the filesystem instead of ROS packages
        """
        r = rospkg.RosPack()
        new_urdf_string = ""
        for line in urdf_string.split('\n'):
            if "package://" in line:
                s = line.split('//')
                s1 = s[1].split('/')
                path = r.get_path(s1[0])
                line = line.replace("package://" + s1[0], path)
            new_urdf_string += line + '\n'

        return self.fix_missing_inertial(new_urdf_string)

    @staticmethod
    def fix_missing_inertial(urdf_string: str) -> str:
        """
        Insert inertial tags for every URDF link that has no inertia.
        This is used to prevent Legacy(PyBullet) from dumping warnings in the terminal

        :param urdf_string: The URDF description as string
        :returns: The new, corrected URDF description as string.
        """

        inertia_tree = ElementTree.ElementTree(ElementTree.Element("inertial"))
        inertia_tree.getroot().append(ElementTree.Element("mass", {"value": "0.1"}))
        inertia_tree.getroot().append(ElementTree.Element("origin", {"rpy": "0 0 0", "xyz": "0 0 0"}))
        inertia_tree.getroot().append(ElementTree.Element("inertia", {"ixx": "0.01",
                                                                      "ixy": "0",
                                                                      "ixz": "0",
                                                                      "iyy": "0.01",
                                                                      "iyz": "0",
                                                                      "izz": "0.01"}))

        # create tree from string
        tree = ElementTree.ElementTree(ElementTree.fromstring(urdf_string))

        for link_element in tree.iter("link"):
            inertial = [*link_element.iter("inertial")]
            if len(inertial) == 0:
                link_element.append(inertia_tree.getroot())

        return ElementTree.tostring(tree.getroot(), encoding='unicode')

    @staticmethod
    def remove_error_tags(urdf_string: str) -> str:
        """
        Removes all tags in the removing_tags list from the URDF since these tags are known to cause errors with the
        URDF_parser

        :param urdf_string: String of the URDF from which the tags should be removed
        :return: The URDF string with the tags removed
        """
        tree = ElementTree.ElementTree(ElementTree.fromstring(urdf_string))
        removing_tags = ["gazebo", "transmission"]
        for tag_name in removing_tags:
            all_tags = tree.findall(tag_name)
            for tag in all_tags:
                tree.getroot().remove(tag)

        return ElementTree.tostring(tree.getroot(), encoding='unicode')

    @staticmethod
    def fix_link_attributes(urdf_string: str) -> str:
        """
        Removes the attribute 'type' from links since this is not parsable by the URDF parser.

        :param urdf_string: The string of the URDF from which the attributes should be removed
        :return: The URDF string with the attributes removed
        """
        tree = ElementTree.ElementTree(ElementTree.fromstring(urdf_string))

        for link in tree.iter("link"):
            if "type" in link.attrib.keys():
                del link.attrib["type"]

        return ElementTree.tostring(tree.getroot(), encoding='unicode')

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
