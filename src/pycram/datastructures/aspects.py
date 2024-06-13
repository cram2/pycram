from .pose import Pose
from typing_extensions import List

from ..designator import ObjectDesignatorDescription


class Aspect:

    def __and__(self, other):
        return AndAspect([self, other])

    def __or__(self, other):
        return OrAspect([self, other])

    def simplify(self):
        pass


class AndAspect(Aspect):

    def __init__(self, aspects: List[Aspect]):
        self.aspects = aspects


class OrAspect(Aspect):

    def __init__(self, aspects: List[Aspect]):
        self.aspects = aspects


class ReachableAspect(Aspect):

    def __init__(self, object_designator: ObjectDesignatorDescription):
        self.object_designator = object_designator

    def reachable(self, pose: Pose) -> bool:
        raise NotImplementedError


class GraspableAspect(Aspect):

    def __init__(self, object_designator: ObjectDesignatorDescription):
        self.object_designator = object_designator

    def graspable(self) -> bool:
        raise NotImplementedError
