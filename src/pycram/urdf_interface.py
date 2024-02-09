import os
import pathlib
import re
from xml.etree import ElementTree

import rospkg
import rospy
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from typing_extensions import Union, List
from urdf_parser_py import urdf
from urdf_parser_py.urdf import (URDF, Collision, Box as URDF_Box, Cylinder as URDF_Cylinder,
                                 Sphere as URDF_Sphere, Mesh as URDF_Mesh)

from pycram.enums import JointType, Shape
from pycram.pose import Pose
from pycram.world import JointDescription as AbstractJointDescription, Joint as AbstractJoint, \
    LinkDescription as AbstractLinkDescription, ObjectDescription as AbstractObjectDescription, World, _is_cached


class LinkDescription(AbstractLinkDescription):
    """
    A class that represents a link description of an object.
    """
    urdf_shape_map = {
        URDF_Box: Shape.BOX,
        URDF_Cylinder: Shape.CYLINDER,
        URDF_Sphere: Shape.SPHERE,
        URDF_Mesh: Shape.MESH
    }

    def __init__(self, urdf_description: urdf.Link):
        super().__init__(urdf_description)

    @property
    def geometry(self) -> Union[Shape, None]:
        """
        Returns the geometry type of the URDF collision element of this link.
        """
        return self.urdf_shape_map[type(self.collision.geometry)]

    @property
    def origin(self) -> Pose:
        return Pose(self.collision.origin.xyz,
                    quaternion_from_euler(*self.collision.origin.rpy))

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


class Joint(AbstractJoint, JointDescription):
    ...


class ObjectDescription(AbstractObjectDescription):
    """
    A class that represents an object description of an object.
    """
    def from_description_file(self, path) -> URDF:
        """
        Create an object description from a description file.

        :param path: The path to the description file.
        """
        with open(path, 'r') as file:
            return URDF.from_xml_string(file.read())

    def preprocess_file(self, path: str, ignore_cached_files: bool) -> str:
        """
        Preprocesses the file, if it is a .obj or .stl file it will be converted to an URDF file.
        """
        path_object = pathlib.Path(path)
        extension = path_object.suffix

        path = _look_for_file_in_data_dir(path, path_object)

        _create_cache_dir_if_not_exists()

        # if file is not yet cached correct the urdf and save if in the cache directory
        if not _is_cached(path) or ignore_cached_files:
            if extension == ".obj" or extension == ".stl":
                path = self._generate_urdf_file(path)
            elif extension == ".urdf":
                with open(path, mode="r") as f:
                    urdf_string = fix_missing_inertial(f.read())
                    urdf_string = remove_error_tags(urdf_string)
                    urdf_string = fix_link_attributes(urdf_string)
                    try:
                        urdf_string = _correct_urdf_string(urdf_string)
                    except rospkg.ResourceNotFound as e:
                        rospy.logerr(f"Could not find resource package linked in this URDF")
                        raise e
                path = World.cache_dir + path_object.name
                with open(path, mode="w") as f:
                    f.write(urdf_string)
            else:  # Using the urdf from the parameter server
                urdf_string = rospy.get_param(path)
                path = World.cache_dir + self.name + ".urdf"
                with open(path, mode="w") as f:
                    f.write(_correct_urdf_string(urdf_string))
        # save correct path in case the file is already in the cache directory
        elif extension == ".obj" or extension == ".stl":
            path = World.cache_dir + path_object.stem + ".urdf"
        elif extension == ".urdf":
            path = World.cache_dir + path_object.name
        else:
            path = World.cache_dir + self.name + ".urdf"

        return path

    def _generate_urdf_file(self, path) -> str:
        """
        Generates an URDf file with the given .obj or .stl file as mesh. In addition, the given rgba_color will be
        used to crate a material tag in the URDF. The resulting file will then be saved in the cach_dir path with
         the name as filename.

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
        urdf_template = fix_missing_inertial(urdf_template)
        rgb = " ".join(list(map(str, self.color.get_rgba())))
        pathlib_obj = pathlib.Path(path)
        path = str(pathlib_obj.resolve())
        content = urdf_template.replace("~a", self.name).replace("~b", path).replace("~c", rgb)
        with open(World.cache_dir + pathlib_obj.stem + ".urdf", "w", encoding="utf-8") as file:
            file.write(content)
        return World.cache_dir + pathlib_obj.stem + ".urdf"

    @property
    def links(self) -> List[LinkDescription]:
        """
        :return: A list of links descriptions of this object.
        """
        return [LinkDescription(link) for link in self.parsed_description.links]

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


def _correct_urdf_string(urdf_string: str) -> str:
    """
    Changes paths for files in the URDF from ROS paths to paths in the file system. Since World (PyBullet legac)
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

    return fix_missing_inertial(new_urdf_string)


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
