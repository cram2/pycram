import inspect
import sys
from dataclasses import dataclass
from typing import Type, List

from ormatic.ormatic import ORMaticExplicitMapping
from ormatic.utils import classproperty
from random_events.utils import recursive_subclasses
from typing_extensions import Optional

from pycrap.ontologies import PhysicalObject
from .casts import StringType
from ..datastructures import grasp
from ..datastructures import pose
from ..datastructures.dataclasses import FrozenObject
from ..datastructures.pose import PoseStamped
from ..designator import ActionDescription
from ..designators import action_designator
from ..language import LanguageNode, SequentialNode, RepeatNode
from ..plan import ActionNode, MotionNode, PlanNode, ResolvedActionNode


# ----------------------------------------------------------------------------------------------------------------------
#            Map all Designators, that are not self-mapping, here.
#            By default all classes are self-mapping, so you only need to add the ones where not every attribute is
#            supposed to be mapped or where an attribute is from a type, which is not mapped itself.
#            Specify the columns(attributes) that are supposed to be tracked in the database.
#            One attribute equals one column. Please refer to the ORMatic documentation for more information.
# ----------------------------------------------------------------------------------------------------------------------


@dataclass
class ActionNodeDAO(ORMaticExplicitMapping):

    @classproperty
    def explicit_mapping(cls) -> Type:
        return ActionNode


@dataclass
class MotionNodeDAO(ORMaticExplicitMapping):

    @classproperty
    def explicit_mapping(cls) -> Type:
        return MotionNode


@dataclass
class ResolvedActionNodeDAO(ORMaticExplicitMapping):
    designator_ref: ActionDescription

    @classproperty
    def explicit_mapping(cls) -> Type:
        return ResolvedActionNode


@dataclass
class FrozenObjectDAO(ORMaticExplicitMapping):
    name: str
    concept: PhysicalObject
    pose: Optional[PoseStamped]

    @classproperty
    def explicit_mapping(cls):
        return FrozenObject


#
# @dataclass
# class ReleaseActionDAO(ActionDescription, ORMaticExplicitMapping):
#     gripper: Arms
#     object_at_execution: Optional[FrozenObject]
#
#     @classproperty
#     def explicit_mapping(cls):
#         return ReleaseAction
#
#
# @dataclass
# class GripActionDAO(ActionDescription, ORMaticExplicitMapping):
#     gripper: Arms
#     effort: float
#     object_at_execution: Optional[FrozenObject]
#
#     @classproperty
#     def explicit_mapping(cls):
#         return GripAction
#
#
# @dataclass
# class ReachToPickUpActionDAO(ActionDescription, ORMaticExplicitMapping):
#     arm: Arms
#     grasp_description: GraspDescription
#     object_at_execution: Optional[FrozenObject]
#
#     @classproperty
#     def explicit_mapping(cls):
#         return ReachToPickUpAction
#
#
# @dataclass
# class PickUpActionDAO(ActionDescription, ORMaticExplicitMapping):
#     arm: Arms
#     object_at_execution: Optional[FrozenObject]
#
#     @classproperty
#     def explicit_mapping(cls):
#         return PickUpAction
#
#
# @dataclass
# class PlaceActionDAO(ActionDescription, ORMaticExplicitMapping):
#     arm: Arms
#     target_location: PoseStamped
#     object_at_execution: Optional[FrozenObject]
#
#     @classproperty
#     def explicit_mapping(cls):
#         return PlaceAction
#
#
# @dataclass
# class TransportActionDAO(ActionDescription, ORMaticExplicitMapping):
#     arm: Arms
#     target_location: PoseStamped
#     object_at_execution: Optional[FrozenObject]
#
#     @classproperty
#     def explicit_mapping(cls):
#         return TransportAction
#
#
# @dataclass
# class DetectActionDAO(ActionDescription, ORMaticExplicitMapping):
#     technique: DetectionTechnique
#     state: Optional[DetectionState]
#     region: Optional[str]
#     object_at_execution: Optional[FrozenObject]
#
#     @classproperty
#     def explicit_mapping(cls):
#         return DetectAction
#
#
# @dataclass
# class GraspingActionDAO(ActionDescription, ORMaticExplicitMapping):
#     arm: Arms
#     object_at_execution: FrozenObject
#
#     @classproperty
#     def explicit_mapping(cls):
#         return GraspingAction


# @dataclass
# class MoveAndPickUpActionDAO(ActionDescription, ORMaticExplicitMapping):
#     standing_position: PoseStamped
#     object_designator: FrozenObject
#     arm: Arms
#     grasp: Grasp
#     keep_joint_states: bool
#
#     @classproperty
#     def explicit_mapping(cls):
#         return MoveAndPickUpAction


# specify custom type mappings
type_mappings = {
    PhysicalObject: StringType(),
}

"""
List of standard classes that are to be mapped to the database.
"""


# pycram.plan.ResolvedActionNode.__annotations__.update({"designator_ref": pycram.designator.ActionDescription})
# pycram.plan.ResolvedActionNode.__annotations__.update({"action": pycram.designator.ActionDescription})
# pycram.plan.MotionNode.__annotations__.update({"designator_ref": pycram.designator.BaseMotion})
# pycram.language.LanguageNode.__annotations__.update({"action": pycram.designator.ActionDescription})

def classes_of_module(module) -> List[Type]:
    result = []
    for name, obj in inspect.getmembers(sys.modules[module.__name__]):
        if inspect.isclass(obj) and obj.__module__ == module.__name__:
            result.append(obj)
    return result


self_mapped_classes = classes_of_module(pose)
self_mapped_classes += [ActionDescription] + classes_of_module(action_designator)
self_mapped_classes += classes_of_module(grasp)

self_mapped_classes += [PlanNode,
                        # LanguageNode,
                        SequentialNode,
                        RepeatNode,
                        # ActionDescription,

                        # ParallelNode,
                        # TryInOrderNode,
                        # TryAllNode,
                        # CodeNode,

                        # MoveTorsoAction,
                        # SetGripperAction,
                        # ParkArmsAction,
                        # NavigateAction,
                        # LookAtAction,
                        # OpenAction,
                        # CloseAction,
                        # FaceAtAction,
                        # SearchAction,
                        # MoveAndPickUpAction
                        ]

# List of all classes that are explicitly mapped above.
explicitly_mapped_classes = recursive_subclasses(ORMaticExplicitMapping)
