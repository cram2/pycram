import os
import pathlib

import numpy as np
from dm_control import mjcf
from geometry_msgs.msg import Point
from typing_extensions import Union, List, Optional, Dict, Tuple
from xml.etree import ElementTree as ET

from ..datastructures.dataclasses import Color, VisualShape, BoxVisualShape, CylinderVisualShape, \
    SphereVisualShape, MeshVisualShape
from ..datastructures.enums import JointType, MJCFGeomType, MJCFJointType
from ..datastructures.pose import Pose
from ..description import JointDescription as AbstractJointDescription, \
    LinkDescription as AbstractLinkDescription, ObjectDescription as AbstractObjectDescription
from ..failures import MultiplePossibleTipLinks
from ..ros.ros_tools import get_parameter

try:
    from multiverse_parser import Configuration, Factory, InertiaSource, GeomBuilder
    from multiverse_parser import (WorldBuilder,
                                   GeomType, GeomProperty,
                                   MeshProperty,
                                   MaterialProperty)
    from multiverse_parser import MjcfExporter
    from pxr import Usd, UsdGeom
except ImportError:
    # do not import this module if multiverse is not found
    raise ImportError("Multiverse not found.")


class LinkDescription(AbstractLinkDescription):
    """
    A class that represents a link description of an object.
    """

    def __init__(self, mjcf_description: mjcf.Element):
        super().__init__(mjcf_description)

    @property
    def geometry(self) -> Union[VisualShape, None]:
        """
        :return: The geometry type of the collision element of this link.
        """
        return self._get_visual_shape(self.parsed_description.find_all('geom')[0])

    @staticmethod
    def _get_visual_shape(mjcf_geometry) -> Union[VisualShape, None]:
        """
        :param mjcf_geometry: The MJCFGeometry to get the visual shape for.
        :return: The VisualShape of the given MJCFGeometry object.
        """
        if mjcf_geometry.type == MJCFGeomType.BOX.value:
            return BoxVisualShape(Color(), [0, 0, 0], mjcf_geometry.size)
        if mjcf_geometry.type == MJCFGeomType.CYLINDER.value:
            return CylinderVisualShape(Color(), [0, 0, 0], mjcf_geometry.size[0], mjcf_geometry.size[1] * 2)
        if mjcf_geometry.type == MJCFGeomType.SPHERE.value:
            return SphereVisualShape(Color(), [0, 0, 0], mjcf_geometry.size[0])
        if mjcf_geometry.type == MJCFGeomType.MESH.value:
            mesh_filename = mjcf_geometry.mesh.file.prefix + mjcf_geometry.mesh.file.extension
            return MeshVisualShape(Color(), [0, 0, 0], mjcf_geometry.mesh.scale, mesh_filename)
        return None

    @property
    def origin(self) -> Union[Pose, None]:
        """
        :return: The origin of this link.
        """
        return parse_pose_from_body_element(self.parsed_description)

    @property
    def name(self) -> str:
        return self.parsed_description.name


class JointDescription(AbstractJointDescription):

    mjcf_type_map = {
        MJCFJointType.HINGE.value: JointType.REVOLUTE,
        MJCFJointType.BALL.value: JointType.SPHERICAL,
        MJCFJointType.SLIDE.value: JointType.PRISMATIC,
        MJCFJointType.FREE.value: JointType.FLOATING
    }
    """
    A dictionary mapping the MJCF joint types to the PyCRAM joint types.
    """

    pycram_type_map = {pycram_type: mjcf_type for mjcf_type, pycram_type in mjcf_type_map.items()}
    """
    A dictionary mapping the PyCRAM joint types to the MJCF joint types.
    """

    def __init__(self, mjcf_description: mjcf.Element, is_virtual: Optional[bool] = False):
        super().__init__(mjcf_description, is_virtual=is_virtual)

    @property
    def origin(self) -> Pose:
        return parse_pose_from_body_element(self.parsed_description)

    @property
    def name(self) -> str:
        return self.parsed_description.name

    @property
    def has_limits(self) -> bool:
        return self.parsed_description.limited

    @property
    def type(self) -> JointType:
        """
        :return: The type of this joint.
        """
        if hasattr(self.parsed_description, 'type'):
            return self.mjcf_type_map[self.parsed_description.type]
        else:
            return self.mjcf_type_map[MJCFJointType.FREE.value]

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
            return self.parsed_description.range[0]
        else:
            return None

    @property
    def upper_limit(self) -> Union[float, None]:
        """
        :return: The upper limit of this joint, or None if the joint has no limits.
        """
        if self.has_limits:
            return self.parsed_description.range[1]
        else:
            return None

    @property
    def parent(self) -> str:
        """
        :return: The name of the parent link of this joint.
        """
        return self._parent_link_element.parent.name

    @property
    def child(self) -> str:
        """
        :return: The name of the child link of this joint.
        """
        return self._parent_link_element.name

    @property
    def _parent_link_element(self) -> mjcf.Element:
        return self.parsed_description.parent

    @property
    def damping(self) -> float:
        """
        :return: The damping of this joint.
        """
        return self.parsed_description.damping

    @property
    def friction(self) -> float:
        raise NotImplementedError("Friction is not implemented for MJCF joints.")


class ObjectFactory(Factory):
    """
    Create MJCF object descriptions from mesh files.
    """
    def __init__(self, object_name: str, file_path: str, config: Configuration, texture_type: str = "png"):
        super().__init__(file_path, config)

        self._world_builder = WorldBuilder(usd_file_path=self.tmp_usd_file_path)

        body_builder = self._world_builder.add_body(body_name=object_name)

        tmp_usd_mesh_file_path, tmp_origin_mesh_file_path = self.import_mesh(
            mesh_file_path=file_path, merge_mesh=True)
        mesh_stage = Usd.Stage.Open(tmp_usd_mesh_file_path)
        for idx, mesh_prim in enumerate([prim for prim in mesh_stage.Traverse() if prim.IsA(UsdGeom.Mesh)]):
            mesh_name = mesh_prim.GetName()
            mesh_path = mesh_prim.GetPath()
            mesh_property = MeshProperty.from_mesh_file_path(mesh_file_path=tmp_usd_mesh_file_path,
                                                             mesh_path=mesh_path)
            # mesh_property._texture_coordinates = None # TODO: See if needed otherwise remove it.
            geom_property = GeomProperty(geom_type=GeomType.MESH,
                                         is_visible=False,
                                         is_collidable=True)
            geom_builder = body_builder.add_geom(geom_name=f"SM_{object_name}_mesh_{idx}",
                                                 geom_property=geom_property)
            geom_builder.add_mesh(mesh_name=mesh_name, mesh_property=mesh_property)

            # Add texture if available
            texture_file_path = file_path.replace(pathlib.Path(file_path).suffix, f".{texture_type}")
            if pathlib.Path(texture_file_path).exists():
                self.add_material_with_texture(geom_builder=geom_builder, material_name=f"M_{object_name}_{idx}",
                                               texture_file_path=texture_file_path)

            geom_builder.build()

        body_builder.compute_and_set_inertial(inertia_source=InertiaSource.FROM_COLLISION_MESH)

    @staticmethod
    def add_material_with_texture(geom_builder: GeomBuilder, material_name: str, texture_file_path: str):
        """
        Add a material with a texture to the geom builder.

        :param geom_builder: The geom builder to add the material to.
        :param material_name: The name of the material.
        :param texture_file_path: The path to the texture file.
        """
        material_property = MaterialProperty(diffuse_color=texture_file_path,
                                             opacity=None,
                                             emissive_color=None,
                                             specular_color=None)
        geom_builder.add_material(material_name=material_name,
                                  material_property=material_property)

    def export_to_mjcf(self, output_file_path: str):
        """
        Export the object to a MJCF file.

        :param output_file_path: The path to the output file.
        """
        exporter = MjcfExporter(self, output_file_path)
        exporter.build()
        exporter.export(keep_usd=False)


class ObjectDescription(AbstractObjectDescription):
    """
    A class that represents an object description of an object.
    """

    COMPILER_TAG = 'compiler'
    """
    The tag of the compiler element in the MJCF file.
    """
    MESH_DIR_ATTR = 'meshdir'
    TEXTURE_DIR_ATTR = 'texturedir'
    """
    The attributes of the compiler element in the MJCF file. The meshdir attribute is the directory where the mesh files
    are stored and the texturedir attribute is the directory where the texture files are stored."""

    class Link(AbstractObjectDescription.Link, LinkDescription):
        ...

    class RootLink(AbstractObjectDescription.RootLink, Link):
        ...

    class Joint(AbstractObjectDescription.Joint, JointDescription):
        ...

    def __init__(self):
        super().__init__()
        self._link_map = None
        self._joint_map = None
        self._child_map = None
        self._parent_map = None
        self._links = None
        self._joints = None
        self.virtual_joint_names = []

    @property
    def child_map(self) -> Dict[str, List[Tuple[str, str]]]:
        """
        :return: A dictionary mapping the name of a link to its children which are represented as a tuple of the child
            joint name and the link name.
        """
        if self._child_map is None:
            self._child_map = self._construct_child_map()
        return self._child_map

    def _construct_child_map(self) -> Dict[str, List[Tuple[str, str]]]:
        """
        Construct the child map of the object.
        """
        child_map = {}
        for joint in self.joints:
            if joint.parent not in child_map:
                child_map[joint.parent] = [(joint.name, joint.child)]
            else:
                child_map[joint.parent].append((joint.name, joint.child))
        return child_map

    @property
    def parent_map(self) -> Dict[str, Tuple[str, str]]:
        """
        :return: A dictionary mapping the name of a link to its parent joint and link as a tuple.
        """
        if self._parent_map is None:
            self._parent_map = self._construct_parent_map()
        return self._parent_map

    def _construct_parent_map(self) -> Dict[str, Tuple[str, str]]:
        """
        Construct the parent map of the object.
        """
        child_map = self.child_map
        parent_map = {}
        for parent, children in child_map.items():
            for child in children:
                parent_map[child[1]] = (child[0], parent)
        return parent_map

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
        Finds the child link and adds a joint to it in the object description.
        for arguments documentation see :meth:`pycram.description.ObjectDescription.add_joint`
        """

        position: Optional[List[float]] = None
        quaternion: Optional[List[float]] = None
        lower_limit: float = 0.0 if lower_limit is None else lower_limit
        upper_limit: float = 0.0 if upper_limit is None else upper_limit
        limit = [lower_limit, upper_limit]

        if origin is not None:
            position = origin.position_as_list()
            quaternion = origin.orientation_as_list()
            quaternion = [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
        if axis is not None:
            axis = [axis.x, axis.y, axis.z]
        self.parsed_description.find(child).add('joint', name=name, type=JointDescription.pycram_type_map[joint_type],
                                                axis=axis, pos=position, quat=quaternion, range=limit)
        if is_virtual:
            self.virtual_joint_names.append(name)

    def load_description(self, path) -> mjcf.RootElement:
        return mjcf.from_file(path, model_dir=pathlib.Path(path).parent)

    def load_description_from_string(self, description_string: str) -> mjcf.RootElement:
        return mjcf.from_xml_string(description_string)

    def generate_from_mesh_file(self, path: str, name: str, color: Optional[Color] = Color(),
                                save_path: Optional[str] = None) -> None:
        """
        Generate a mjcf xml file with the given .obj or .stl file as mesh. In addition, use the given rgba_color
         to create a material tag in the xml.

        :param path: The path to the mesh file.
        :param name: The name of the object.
        :param color: The color of the object.
        :param save_path: The path to save the generated xml file.
        """
        factory = ObjectFactory(object_name=name, file_path=path,
                                config=Configuration(model_name=name,
                                                     fixed_base=False,
                                                     default_rgba=np.array(color.get_rgba())))
        factory.export_to_mjcf(output_file_path=save_path)

    def generate_from_description_file(self, path: str, save_path: str, make_mesh_paths_absolute: bool = True) -> None:
        model_str = self.replace_relative_paths_with_absolute_paths(path)
        self.write_description_to_file(model_str, save_path)

    def replace_relative_paths_with_absolute_paths(self, model_path: str) -> str:
        """
        Replace the relative paths in the xml file to be absolute paths.

        :param model_path: The path to the xml file.
        """
        tree = ET.parse(model_path)
        root = tree.getroot()
        compiler = root.find(self.COMPILER_TAG)
        model_dir = pathlib.Path(model_path).parent
        for rel_dir_attrib in [self.MESH_DIR_ATTR, self.TEXTURE_DIR_ATTR]:
            rel_dir = compiler.get(rel_dir_attrib)
            abs_dir = str(pathlib.Path(os.path.join(model_dir, rel_dir)).resolve())
            compiler.set(rel_dir_attrib, abs_dir)
        return ET.tostring(root, encoding='unicode', method='xml')

    def generate_from_parameter_server(self, name: str, save_path: str) -> None:
        mjcf_string = get_parameter(name)
        self.write_description_to_file(mjcf_string, save_path)

    @property
    def joints(self) -> List[JointDescription]:
        """
        :return: A list of joints descriptions of this object.
        """
        if self._joints is None:
            self._joints = [JointDescription(joint) for joint in self.parsed_description.find_all('joint')]
        return self._joints

    @property
    def links(self) -> List[LinkDescription]:
        """
        :return: A list of link descriptions of this object.
        """
        if self._links is None:
            self._links = [LinkDescription(link) for link in self.parsed_description.find_all('body')]
        return self._links

    def get_root(self) -> str:
        """
        :return: the name of the root link of this object.
        """
        if len(self.links) == 1:
            return self.links[0].name
        elif len(self.links) > 1:
            return self.links[1].name
        else:
            raise ValueError("No links found in the object description.")

    def get_tip(self) -> str:
        """
        :return: the name of the tip link of this object.
        :raises MultiplePossibleTipLinks: If there are multiple possible tip links.
        """
        link = self.get_root()
        while link in self.child_map:
            children = self.child_map[link]
            if len(children) > 1:
                # Multiple children, can't decide which one to take (e.g. fingers of a hand)
                raise MultiplePossibleTipLinks(self.name, link, [child[1] for child in children])
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
        :param fixed: Whether to include fixed joints in the chain (Note: not used in MJCF).
        :return: the chain of links from 'start_link_name' to 'end_link_name'.
        """
        chain = []
        if links:
            chain.append(end_link_name)
        link = end_link_name
        while link != start_link_name:
            (joint, parent) = self.parent_map[link]
            if joints:
                chain.append(joint)
            if links:
                chain.append(parent)
            link = parent
        chain.reverse()
        return chain

    @staticmethod
    def get_file_extension() -> str:
        """
        :return: The file extension of the URDF file.
        """
        return '.xml'

    @property
    def origin(self) -> Pose:
        return parse_pose_from_body_element(self.parsed_description)

    @property
    def name(self) -> str:
        return self.parsed_description.name


def parse_pose_from_body_element(body: mjcf.Element) -> Pose:
    """
    Parse the pose from a body element.

    :param body: The body element.
    :return: The pose of the body.
    """
    position = body.pos
    quaternion = body.quat
    position = [0, 0, 0] if position is None else position
    quaternion = [1, 0, 0, 0] if quaternion is None else quaternion
    quaternion = [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
    return Pose(position, quaternion)
