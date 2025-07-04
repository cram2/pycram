from pycram.robot_plans.actions.base import ActionDescription
from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

from pycram.has_parameters import has_parameters
from pycram.plan import with_plan

from pycram.datastructures.partial_designator import PartialDesignator

from typing_extensions import Union, Optional, Type, Any, Iterable

from pycrap.ontologies import Location
from pycram.robot_plans.motions.motion_designator import DetectingMotion
from pycram.failure_handling import try_action
from pycram.failures import PerceptionObjectNotFound

from pycram.datastructures.enums import DetectionTechnique, DetectionState

from pycram.world_concepts.world_object import Object


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

    # object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
    # """
    # The object at the time this Action got created. It is used to be a static, information holding entity
    # """

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

        # # Store the object's data copy at execution
        # self.pre_perform(record_object_pre_perform)

    def plan(self) -> None:
        return try_action(DetectingMotion(technique=self.technique, state=self.state,
                                          object_designator_description=self.object_designator,
                                          region=self.region), PerceptionObjectNotFound)

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        return
        # if not result:
        #     raise PerceptionObjectNotFound(self.object_designator, self.technique, self.region)

    @classmethod
    @with_plan
    def description(cls, technique: Union[Iterable[DetectionTechnique], DetectionTechnique],
                    state: Union[Iterable[DetectionState], DetectionState] = None,
                    object_designator: Union[Iterable[Object], Object] = None,
                    region: Union[Iterable[Location], Location] = None) -> PartialDesignator[Type[DetectAction]]:
        return PartialDesignator(DetectAction, technique=technique,
                                 state=state,
                                 object_designator=object_designator,
                                 region=region)

# @has_parameters
# @dataclass
# class MoveAndPickUpAction(ActionDescription):
#     """
#     Navigate to `standing_position`, then turn towards the object and pick it up.
#     """
#
#     standing_position: PoseStamped
#     """
#     The pose to stand before trying to pick up the object
#     """
#
#     object_designator: Object
#     """
#     The object to pick up
#     """
#
#     arm: Arms
#     """
#     The arm to use
#     """
#
#     grasp_description: GraspDescription
#     """
#     The grasp to use
#     """
#
#     keep_joint_states: bool = ActionConfig.navigate_keep_joint_states
#     """
#     Keep the joint states of the robot the same during the navigation.
#     """
#
#     object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
#     """
#     The object at the time this Action got created. It is used to be a static, information holding entity. It is
#     not updated when the BulletWorld object is changed.
#     """
#
#     _pre_perform_callbacks = []
#     """
#     List to save the callbacks which should be called before performing the action.
#     """
#
#     def __post_init__(self):
#         super().__post_init__()
#
#         # Store the object's data copy at execution
#         self.pre_perform(record_object_pre_perform)
#
#     def plan(self):
#         NavigateAction(self.standing_position, self.keep_joint_states).perform()
#         FaceAtAction(self.object_designator.pose, self.keep_joint_states).perform()
#         PickUpAction(self.object_designator, self.arm, self.grasp_description).perform()
#
#     def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
#         # The validation will be done in each of the core action perform methods so no need to validate here.
#         pass
#
#     @classmethod
#     @with_plan
#     def description(cls, standing_position: Union[Iterable[PoseStamped], PoseStamped],
#                     object_designator: Union[Iterable[PoseStamped], PoseStamped],
#                     arm: Union[Iterable[Arms], Arms] = None,
#                     grasp_description: Union[Iterable[Grasp], Grasp] = None,
#                     keep_joint_states: Union[Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states) -> \
#             PartialDesignator[Type[MoveAndPickUpAction]]:
#         return PartialDesignator(MoveAndPickUpAction,
#                                  standing_position=standing_position,
#                                  object_designator=object_designator,
#                                  arm=arm,
#                                  grasp_description=grasp_description,
#                                  keep_joint_states=keep_joint_states)


DetectActionDescription = DetectAction.description

# MoveAndPickUpActionDescription = MoveAndPickUpAction.description

