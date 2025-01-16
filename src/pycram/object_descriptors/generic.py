from typing import Optional, Tuple

from typing_extensions import List, Any, Union, Dict

from geometry_msgs.msg import Point

from ..datastructures.dataclasses import VisualShape, BoxVisualShape, Color
from ..datastructures.enums import JointType
from ..datastructures.pose import Pose
from ..description import JointDescription as AbstractJointDescription, LinkDescription as AbstractLinkDescription, \
    ObjectDescription as AbstractObjectDescription


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
    def origin(self) -> Pose:
        return Pose(self.parsed_description.visual_frame_position)

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
        return Point(0, 0, 0)

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
    def origin(self) -> Pose:
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
        self._links = [LinkDescription(*args, **kwargs)]

    def load_description(self, path: str) -> Any:
        ...

    @classmethod
    def generate_from_mesh_file(cls, path: str, name: str, save_path: str) -> str:
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
                  axis: Point, parent: Optional[str] = None, origin: Optional[Pose] = None,
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
    def origin(self) -> Pose:
        return self._links[0].origin

    @property
    def name(self) -> str:
        return self._links[0].name
