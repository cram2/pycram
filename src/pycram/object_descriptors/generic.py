from typing import Optional, Tuple

from trimesh import Trimesh
from typing_extensions import List, Any, Union, Dict, Self, TYPE_CHECKING

from .urdf import ObjectDescription as UrdfObjectDescription
from ..config.world_conf import WorldConfig
from ..datastructures.dataclasses import VisualShape, BoxVisualShape, Color, AxisAlignedBoundingBox, RotatedBoundingBox, \
    BoundingBox
from ..datastructures.enums import JointType
from ..datastructures.pose import PoseStamped, Point
from ..description import JointDescription as AbstractJointDescription, LinkDescription as AbstractLinkDescription, \
    ObjectDescription as AbstractObjectDescription, ObjectDescription

if TYPE_CHECKING:
    pass


class NamedBoxVisualShape(BoxVisualShape):
    def __init__(self, name: str, color: Color, visual_frame_position: List[float], half_extents: List[float]):
        super().__init__(color, visual_frame_position, half_extents)
        self._name: str = name

    @property
    def name(self) -> str:
        return self._name


class LinkDescription(AbstractLinkDescription):

    def __init__(self, name: str, visual_frame_position: List[float], half_extents: List[float],
                 color: Color = Color()):
        super().__init__(NamedBoxVisualShape(name, color, visual_frame_position, half_extents))

    @property
    def geometry(self) -> Union[VisualShape, None]:
        return self.parsed_description

    @property
    def origin(self) -> PoseStamped:
        return PoseStamped.from_list(self.parsed_description.visual_frame_position)

    @property
    def name(self) -> str:
        return self.parsed_description.name

    @property
    def color(self) -> Color:
        return self.parsed_description.rgba_color


class JointDescription(AbstractJointDescription):

    @property
    def parent(self) -> str:
        raise NotImplementedError

    @property
    def child(self) -> str:
        raise NotImplementedError

    @property
    def type(self) -> JointType:
        return JointType.UNKNOWN

    @property
    def axis(self) -> Point:
        return Point(x=0, y=0, z=0)

    @property
    def has_limits(self) -> bool:
        return False

    @property
    def lower_limit(self) -> Union[float, None]:
        return 0

    @property
    def upper_limit(self) -> Union[float, None]:
        return 0

    @property
    def parent_link_name(self) -> str:
        raise NotImplementedError

    @property
    def child_link_name(self) -> str:
        raise NotImplementedError

    @property
    def origin(self) -> PoseStamped:
        raise NotImplementedError

    @property
    def name(self) -> str:
        raise NotImplementedError


class ObjectDescription(AbstractObjectDescription):
    """
    A generic description of an object in the environment. This description can be applied to any object.
    The current use case involves perceiving objects using RoboKudo and spawning them with specified size and color.
    """

    class Link(AbstractObjectDescription.Link, LinkDescription):
        ...

    class RootLink(AbstractObjectDescription.RootLink, Link):
        ...

    class Joint(AbstractObjectDescription.Joint, JointDescription):
        ...

    def __init__(self, *args, **kwargs):
        super().__init__()
        self._links = [LinkDescription(*args, **kwargs)]

    def merge_description(self, other: ObjectDescription, parent_link: Optional[str] = None,
                          child_link: Optional[str] = None,
                          joint_type: JointType = JointType.FIXED,
                          axis: Optional[Point] = None,
                          lower_limit: Optional[float] = None, upper_limit: Optional[float] = None,
                          child_pose_wrt_parent: Optional[PoseStamped] = None,
                          in_place: bool = False,
                          new_description_file: Optional[str] = None) -> Union[ObjectDescription, Self]:
        return self.merge_using_bounding_boxes(other, child_pose_wrt_parent, new_description_file)

    def merge_using_bounding_boxes(self, other: ObjectDescription, child_pose_wrt_parent: Optional[PoseStamped] = None,
                                   new_description_file: Optional[str] = None) -> Union[ObjectDescription, Self]:
        """
        Merge the current object description with another object description by merging their bounding boxes and
        creating a URDF description with the new merged bounding box/ mesh.

        :param other: The other object description to merge with.
        :param child_pose_wrt_parent: The pose of the child object wrt the parent object.
        :param new_description_file: The path to save the new description file.
        :return: The new object description.
        """
        my_bounding_box: AxisAlignedBoundingBox = self._links[0].geometry.get_axis_aligned_bounding_box()
        other_bounding_box: AxisAlignedBoundingBox = other._links[0].geometry.get_axis_aligned_bounding_box()
        transform = child_pose_wrt_parent.to_transform_stamped(self.name)
        other_bounding_box: RotatedBoundingBox = other_bounding_box.get_rotated_box(transform)
        new_mesh = BoundingBox.merge_multiple_bounding_boxes_into_mesh([my_bounding_box, other_bounding_box],
                                                                       use_random_events=False)
        return self.create_urdf_from_mesh(new_mesh, path=new_description_file)

    def merge_using_urdf(self, other: ObjectDescription, child_pose_wrt_parent: Optional[PoseStamped] = None,
                         new_description_file: Optional[str] = None) -> Union[ObjectDescription, Self]:
        """
        Merge the current object description with another object description by creating a URDF description for both
        objects and merging them.

        :param other: The other object description to merge with.
        :param child_pose_wrt_parent: The pose of the child object wrt the parent object.
        :param new_description_file: The path to save the new description file.
        :return: The new object description.
        """
        my_urdf_description = self.create_urdf_from_mesh()
        other_urdf_description = other.create_urdf_from_mesh()
        return my_urdf_description.merge_description(other_urdf_description,
                                                     child_pose_wrt_parent=child_pose_wrt_parent,
                                                     new_description_file=new_description_file)

    def create_urdf_from_mesh(self, mesh: Optional[Trimesh] = None,
                              mesh_file_path: Optional[str] = None,
                              path: Optional[str] = None) -> UrdfObjectDescription:
        """
        Create a URDF description from the mesh and the current object description.

        :param mesh: The mesh to create the URDF description from.
        :param mesh_file_path: The path to the mesh file.
        :param path: The path to save the URDF description.
        :return: The URDF object description.
        """
        mesh = mesh if mesh else self.get_mesh()
        mesh_file_path = mesh_file_path if mesh_file_path else f"{WorldConfig.cache_dir}/{self.name}.obj"
        mesh.export(mesh_file_path)
        path = f"{WorldConfig.cache_dir}/{self.name}.urdf" if not path else path
        new_description = UrdfObjectDescription()
        new_description.generate_from_mesh_file(mesh_file_path, self.name, path)
        new_description.update_description_from_file(path)
        return new_description

    def get_mesh(self) -> Trimesh:
        """
        Get the mesh of the object.
        """
        return self._links[0].geometry.get_axis_aligned_bounding_box().as_mesh

    def load_description(self, path: str) -> Any:
        ...

    @classmethod
    def generate_from_mesh_file(cls, path: str, name: str, save_path: str, color: Color) -> str:
        raise NotImplementedError

    @classmethod
    def generate_from_description_file(cls, path: str, save_path: str, make_mesh_paths_absolute: bool = True) -> str:
        raise NotImplementedError

    @classmethod
    def generate_from_parameter_server(cls, name: str, save_path: str) -> str:
        raise NotImplementedError

    @property
    def parent_map(self) -> Dict[str, Tuple[str, str]]:
        return {}

    @property
    def link_map(self) -> Dict[str, LinkDescription]:
        return {self._links[0].name: self._links[0]}

    @property
    def joint_map(self) -> Dict[str, JointDescription]:
        return {}

    @property
    def child_map(self) -> Dict[str, List[Tuple[str, str]]]:
        return {}

    def add_joint(self, name: str, child: str, joint_type: JointType,
                  axis: Point, parent: Optional[str] = None, origin: Optional[PoseStamped] = None,
                  lower_limit: Optional[float] = None, upper_limit: Optional[float] = None,
                  is_virtual: Optional[bool] = False) -> None:
        ...

    @property
    def shape_data(self) -> List[float]:
        return self._links[0].geometry.shape_data()['halfExtents']

    @property
    def color(self) -> Color:
        return self._links[0].color

    @property
    def links(self) -> List[LinkDescription]:
        return self._links

    def get_link_by_name(self, link_name: str) -> LinkDescription:
        if link_name == self._links[0].name:
            return self._links[0]

    @property
    def joints(self) -> List[JointDescription]:
        return []

    def get_joint_by_name(self, joint_name: str) -> JointDescription:
        ...

    def get_root(self) -> str:
        return self._links[0].name

    def get_chain(self, start_link_name: str, end_link_name: str, joints: Optional[bool] = True,
                  links: Optional[bool] = True, fixed: Optional[bool] = True) -> List[str]:
        raise NotImplementedError("Do Not Do This on generic objects as they have no chains")

    @staticmethod
    def get_file_extension() -> str:
        raise NotImplementedError("Do Not Do This on generic objects as they have no extensions")

    @property
    def origin(self) -> PoseStamped:
        return self._links[0].origin

    @property
    def name(self) -> str:
        return self._links[0].name
