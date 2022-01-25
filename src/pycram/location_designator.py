from .designator import Designator, DesignatorError, DesignatorDescription
from typing import List

from .object_designator import ObjectDesignator


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
        return "ObjectDesignator({})".format(self._description.__dict__)


class LocationDesignatorDescription(DesignatorDescription):
    pose: List[float]

    def __init__(self, pose=None, resolver="grounding"):
        super().__init__(resolver)
        self.pose = pose


class ObjectRelativeLocationDesignatorDescription(LocationDesignatorDescription):
    relative_pose: List[float]
    reference_object: ObjectDesignator
    timestamp: float

    def __init__(self, relative_pose: List[float] = None, reference_object: ObjectDesignator = None,
                 resolver="grounding"):
        super().__init__(pose=None, resolver=resolver)
        self.relative_pose = relative_pose
        self.reference_object = reference_object
