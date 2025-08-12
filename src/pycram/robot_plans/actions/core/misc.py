from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

from typing_extensions import Union, Optional, Type, Any, Iterable

from ...motions.misc import DetectingMotion
from ....datastructures.enums import DetectionTechnique, DetectionState
from ....datastructures.partial_designator import PartialDesignator
from ....failure_handling import try_action
from ....failures import PerceptionObjectNotFound
from ....has_parameters import has_parameters
from ....robot_plans.actions.base import ActionDescription
from ....world_concepts.world_object import Object
from pycrap.ontologies import Location


@has_parameters
@dataclass
class DetectAction(ActionDescription):
    """
    Detects an object that fits the object description and returns an object designator_description describing the object.

    If no object is found, an PerceptionObjectNotFound error is raised.
    """

    technique: DetectionTechnique
    """
    The technique that should be used for detection
    """
    state: Optional[DetectionState] = None
    """
    The state of the detection, e.g Start Stop for continues perception
    """
    object_designator: Optional[Object] = None
    """
    The type of the object that should be detected, only considered if technique is equal to Type
    """
    region: Optional[Location] = None
    """
    The region in which the object should be detected
    """

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()


    def plan(self) -> None:
        return try_action(DetectingMotion(technique=self.technique, state=self.state,
                                          object_designator_description=self.object_designator,
                                          region=self.region), PerceptionObjectNotFound)

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        return
        # if not result:
        #     raise PerceptionObjectNotFound(self.object_designator, self.technique, self.region)

    @classmethod
    def description(cls, technique: Union[Iterable[DetectionTechnique], DetectionTechnique],
                    state: Union[Iterable[DetectionState], DetectionState] = None,
                    object_designator: Union[Iterable[Object], Object] = None,
                    region: Union[Iterable[Location], Location] = None) -> PartialDesignator[Type[DetectAction]]:
        return PartialDesignator(DetectAction, technique=technique,
                                 state=state,
                                 object_designator=object_designator,
                                 region=region)

DetectActionDescription = DetectAction.description

