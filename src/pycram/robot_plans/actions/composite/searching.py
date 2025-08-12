from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

from typing_extensions import Union, Optional, Type, Any, Iterable

from pycrap.ontologies import PhysicalObject
from ..core.misc import DetectActionDescription
from ..core.navigation import LookAtActionDescription, NavigateActionDescription
from ....datastructures.enums import DetectionTechnique
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....datastructures.world import World
from ....designators.location_designator import CostmapLocation
from ....designators.object_designator import BelieveObject
from ....failures import PerceptionObjectNotFound
from ....has_parameters import has_parameters
from ....language import TryInOrderPlan, SequentialPlan
from ....local_transformer import LocalTransformer
from ....robot_plans.actions.base import ActionDescription


@has_parameters
@dataclass
class SearchAction(ActionDescription):
    """
    Searches for a target object around the given location.
    """

    target_location: PoseStamped
    """
    Location around which to look for a target object.
    """

    object_type: Type[PhysicalObject]
    """
    Type of the object which is searched for.
    """

    def plan(self) -> None:
        NavigateActionDescription(
            CostmapLocation(target=self.target_location, visible_for=World.robot)).resolve().perform()

        lt = LocalTransformer()
        target_base = lt.transform_pose(self.target_location, World.robot.tf_frame)

        target_base_left = target_base.copy()
        target_base_left.pose.position.y -= 0.5

        target_base_right = target_base.copy()
        target_base_right.pose.position.y += 0.5

        plan = TryInOrderPlan(self.context,
            SequentialPlan(
                self.context,
                LookAtActionDescription(target_base_left),
                DetectActionDescription(DetectionTechnique.TYPES,
                                        object_designator=BelieveObject(types=[self.object_type]))),
            SequentialPlan(
                self.context,
                LookAtActionDescription(target_base_right),
                DetectActionDescription(DetectionTechnique.TYPES,
                                        object_designator=BelieveObject(types=[self.object_type]))),
            SequentialPlan(self.context,
                LookAtActionDescription(target_base),
                DetectActionDescription(DetectionTechnique.TYPES,
                                        object_designator=BelieveObject(types=[self.object_type]))))

        obj = plan.perform()
        if obj is not None:
            return obj
        raise PerceptionObjectNotFound(self.object_type, DetectionTechnique.TYPES, self.target_location)

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        pass

    @classmethod
    def description(cls, target_location: Union[Iterable[PoseStamped], PoseStamped],
                    object_type: Union[Iterable[PhysicalObject], PhysicalObject]) -> PartialDesignator[
        Type[SearchAction]]:
        return PartialDesignator(SearchAction, target_location=target_location, object_type=object_type)


SearchActionDescription = SearchAction.description
