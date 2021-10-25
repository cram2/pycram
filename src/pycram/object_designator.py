from .designator import Designator, DesignatorError, DesignatorDescription
from typing import List


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

    def __init__(self, type_: str = None, name: str = None, resolver="grounding"):
        super().__init__(resolver)
        self.type = type_
        self.name = name


class LocatedObjectDesignatorDescription(ObjectDesignatorDescription):
    pose: List[float]
    reference_frame: str
    timestamp: float

    def __init__(self, type_: str = None, name: str = None, pose: List[float] = None, reference_frame: str = "world",
                 timestamp: float = None, resolver="grounding"):
        super().__init__(type_, name, resolver)
        self.pose = pose
        self.reference_frame = reference_frame
        self.timestamp = timestamp
