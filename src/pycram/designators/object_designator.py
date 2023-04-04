from ..designator import Designator, DesignatorError, DesignatorDescription
from ..bullet_world import Object
from typing import List, Tuple, Union


class ObjectDesignator(Designator):

    resolvers = {}

    def __init__(self, description, parent=None):
        super().__init__(description, parent)

    def next_solution(self):
        try:
            self.reference()
        except DesignatorError:
            pass

        if self._solutions.has(self._index + 1):
            desig = ObjectDesignator(self._description, self)
            desig._solutions = self._solutions
            desig._index = self._index + 1
            return desig

        return None

    def ground(self):
        return None

    def get_slots(self):
        return self._description.get_slots()

    def __str__(self):
        return "ObjectDesignator({})".format(self._description.__dict__)


class ObjectDesignatorDescription(DesignatorDescription):
    type: str
    name: str
    object: Object
    pose: Tuple[List[float], List[float]]

    def __init__(self, type: str = None, name: str = None, object: Object = None, resolver="grounding"):
        super().__init__(resolver)
        self.type = type
        self.name = name
        self.object = object
        self.pose = None

class BelieveObject(ObjectDesignatorDescription):

    def __init__(type, name, object, resolver="grounding"):
        super().__init__(type, name, object)

class ObjectPart(ObjectDesignatorDescription):
    type: str
    name: str
    part_of: Union[Object, ObjectDesignator]

    def __init__(self, type: str, name: str, part_of: Object = None, resolver="grounding"):
        super().__init__(type, name, resolver)
        #self.type = type
        #self.name = name
        self.part_of = part_of


class LocatedObject(ObjectDesignatorDescription):
    pose: List[float]
    reference_frame: str
    timestamp: float

    def __init__(self, type: str = None, name: str = None, pose: List[float] = None, reference_frame: str = "world",
                 timestamp: float = None, resolver="grounding"):
        super().__init__(type, name, resolver)
        self.pose = pose
        self.reference_frame = reference_frame
        self.timestamp = timestamp
