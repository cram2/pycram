from typing import List, Tuple, Union

from .object_designator import ObjectDesignator
from ..bullet_world import Object
from ..designator import Designator, DesignatorError, DesignatorDescription


class LocationDesignator(Designator):

    resolvers = {}

    def __init__(self, description, parent=None):
        super().__init__(description, parent)

    def next_solution(self):
        try:
            self.reference()
        except DesignatorError:
            pass

        if self._solutions.has(self._index + 1):
            desig = LocationDesignator(self._description, self)
            desig._solutions = self._solutions
            desig._index = self._index + 1
            return desig

        return None

    def ground(self):
        return None

    def get_slots(self):
        return self._description.get_slots()

    def __str__(self):
        return "LocationDesignator({})".format(self._description.__dict__)


class LocationDesignatorDescription(DesignatorDescription):
    pose: List[float]

    def __init__(self, pose=None, resolver="grounding"):
        super().__init__(resolver)
        self.pose = pose

class Location(LocationDesignatorDescription):

    def __init__(self, position, orientation, resolver=grounding):
        super().__init__(pose=None, resolver=resolver)
        self.position = position
        self.orientation = Orientation
        self.pose = [position, orientation]


class ObjectRelativeLocation(LocationDesignatorDescription):
    relative_pose: List[float]
    reference_object: ObjectDesignator
    timestamp: float

    def __init__(self, relative_pose: List[float] = None, reference_object: ObjectDesignator = None,
                 resolver="grounding"):
        super().__init__(pose=None, resolver=resolver)
        self.relative_pose = relative_pose
        self.reference_object = reference_object

class CostmapLocation(LocationDesignatorDescription):
    reachable_for: Object
    visible_for: Object
    target: Union[Tuple[List[float], List[float]], Object]

    def __init__(self, target, reachable_for=None, visible_for=None, reachable_arm=None):
        super().__init__(pose=None, resolver="costmap")
        self.target = target
        self.reachable_for = reachable_for
        self.visible_for = visible_for
        self.reachable_arm = reachable_arm


class SemanticCostmapLocation(LocationDesignatorDescription):
    urdf_link_name: str
    part_of: Object
    for_object: Object

    def __init__(self, urdf_link_name, part_of, for_object=None):
        super().__init__(pose=None, resolver="semantic-costmap")
        self.urdf_link_name = urdf_link_name
        self.part_of = part_of
        self.for_object = for_object
