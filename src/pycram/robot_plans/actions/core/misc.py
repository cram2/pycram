from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

from semantic_digital_twin.spatial_types import TransformationMatrix
from semantic_digital_twin.world_description.geometry import BoundingBox
from semantic_digital_twin.world_description.world_entity import Region, \
    SemanticAnnotation, SemanticEnvironmentAnnotation
from typing_extensions import Union, Optional, Type, Any, Iterable

from perception import PerceptionQuery
from ....datastructures.enums import DetectionTechnique, DetectionState
from ....datastructures.partial_designator import PartialDesignator
from ....has_parameters import has_parameters
from ....robot_plans.actions.base import ActionDescription


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
    object_sem_annotation: Optional[Type[SemanticAnnotation]] = None
    """
    The type of the object that should be detected, only considered if technique is equal to Type
    """
    region: Optional[Region] = None
    """
    The region in which the object should be detected
    """

    def __post_init__(self):
        super().__post_init__()

    def plan(self) -> None:
        if not self.object_sem_annotation and self.region:
            raise AttributeError("Either a Semantic Annotation or a Region must be provided.")
        region_bb = self.region.area.as_bounding_box_collection_in_frame(
            self.robot_view.root).bounding_box if self.region else BoundingBox(
            origin=TransformationMatrix(reference_frame=self.robot_view.root), min_x=-1, min_y=-1, min_z=0, max_x=3,
            max_y=3, max_z=3)
        if not self.object_sem_annotation:
            self.object_sem_annotation = SemanticEnvironmentAnnotation
        query = PerceptionQuery(self.object_sem_annotation, region_bb, self.robot_view, self.world)

        return query.from_world()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        return
        # if not result:
        #     raise PerceptionObjectNotFound(self.object_designator, self.technique, self.region)

    @classmethod
    def description(cls, technique: Union[Iterable[DetectionTechnique], DetectionTechnique],
                    state: Union[Iterable[DetectionState], DetectionState] = None,
                    object_sem_annotation: Union[Iterable[Type[SemanticAnnotation]], Type[SemanticAnnotation]] = None,
                    region: Union[Iterable[Region], Region] = None) -> PartialDesignator[Type[DetectAction]]:
        return PartialDesignator(DetectAction, technique=technique,
                                 state=state,
                                 object_sem_annotation=object_sem_annotation,
                                 region=region)


DetectActionDescription = DetectAction.description
