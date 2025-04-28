from dataclasses import dataclass
from datetime import datetime
from typing import Type

from ormatic.ormatic import ORMaticExplicitMapping
from ormatic.utils import classproperty
from typing_extensions import Optional

from pycrap.ontologies import PhysicalObject
from .casts import StringType
from ..datastructures.dataclasses import FrozenObject as FrozenObjectIm
from ..datastructures.enums import Arms, DetectionTechnique, DetectionState, Grasp, TaskStatus
from ..datastructures.grasp import GraspDescription
from ..datastructures.pose import Pose, PoseStamped, Vector3
from ..designators.action_designator import MoveTorsoAction, SetGripperAction, ReleaseAction as ReleaseActionDesignator,\
    GripAction as GripActionDesignator, ParkArmsAction , PickUpAction as PickUpActionDesignator, \
    PlaceAction as PlaceActionDesignator, NavigateAction, TransportAction as TransportActionDesignator, LookAtAction, \
    DetectAction as DetectActionDesignator, OpenAction, CloseAction, GraspingAction as GraspingActionDesignator, \
    FaceAtAction, ActionDescription, ReachToPickUpAction as ReachToPickUpActionDesignator
from pycram.plan import ActionNode, MotionNode, ResolvedActionNode



# ----------------------------------------------------------------------------------------------------------------------
#            Map all Designators, that are not self-mapping, here.
#            By default all classes are self-mapping, so you only need to add the ones where not every attribute is
#            supposed to be mapped or where an attribute is from a type, which is not mapped itself.
#            Specify the columns(attributes) that are supposed to be tracked in the database.
#            One attribute equals one column. Please refer to the ORMatic documentation for more information.
# ----------------------------------------------------------------------------------------------------------------------

@dataclass
class TaskTreeNode:
    start_time: datetime
    end_time: Optional[datetime]
    status: TaskStatus
    reason: Optional[str]
    action: Optional[ActionDescription] = None
    parent: Optional["TaskTreeNode"] = None

@dataclass
class ORMActionNode(ORMaticExplicitMapping):

    @classproperty
    def explicit_mapping(cls) -> Type:
        return ActionNode


@dataclass
class ORMMotionNode(ORMaticExplicitMapping):

    @classproperty
    def explicit_mapping(cls) -> Type:
        return MotionNode


@dataclass
class ORMResolvedActionNode(ORMaticExplicitMapping):

    designator_ref: ActionDescription

    @classproperty
    def explicit_mapping(cls) -> Type:
        return ResolvedActionNode


@dataclass
class FrozenObject(ORMaticExplicitMapping):
    name: str
    concept: PhysicalObject
    pose: Optional[PoseStamped]

    @classproperty
    def explicit_mapping(cls):
        return FrozenObjectIm


@dataclass
class ReleaseAction(ActionDescription, ORMaticExplicitMapping):
    gripper: Arms
    object_at_execution: Optional[FrozenObject]

    @classproperty
    def explicit_mapping(cls):
        return ReleaseActionDesignator


@dataclass
class GripAction(ActionDescription, ORMaticExplicitMapping):
    gripper: Arms
    effort: float
    object_at_execution: Optional[FrozenObject]

    @classproperty
    def explicit_mapping(cls):
        return GripActionDesignator


@dataclass
class ReachToPickUpAction(ActionDescription, ORMaticExplicitMapping):
    arm: Arms
    prepose_distance: float
    grasp_description: GraspDescription
    object_at_execution: Optional[FrozenObject]

    @classproperty
    def explicit_mapping(cls):
        return ReachToPickUpActionDesignator


@dataclass
class PickUpAction(ActionDescription, ORMaticExplicitMapping):
    arm: Arms
    prepose_distance: float
    grasp_description: GraspDescription
    object_at_execution: Optional[FrozenObject]

    @classproperty
    def explicit_mapping(cls):
        return PickUpActionDesignator


@dataclass
class PlaceAction(ActionDescription, ORMaticExplicitMapping):
    arm: Arms
    target_location: PoseStamped
    object_at_execution: Optional[FrozenObject]

    @classproperty
    def explicit_mapping(cls):
        return PlaceActionDesignator


@dataclass
class TransportAction(ActionDescription, ORMaticExplicitMapping):
    arm: Arms
    target_location: PoseStamped
    pickup_prepose_distance: float
    object_at_execution: Optional[FrozenObject]

    @classproperty
    def explicit_mapping(cls):
        return TransportActionDesignator


@dataclass
class DetectAction(ActionDescription, ORMaticExplicitMapping):
    technique: DetectionTechnique
    state: Optional[DetectionState]
    region: Optional[str]
    object_at_execution: Optional[FrozenObject]

    @classproperty
    def explicit_mapping(cls):
        return DetectActionDesignator


@dataclass
class GraspingAction(ActionDescription, ORMaticExplicitMapping):
    arm: Arms
    prepose_distance: float
    object_at_execution: FrozenObject

    @classproperty
    def explicit_mapping(cls):
        return GraspingActionDesignator


@dataclass
class MoveAndPickUpAction(ActionDescription, ORMaticExplicitMapping):
    standing_position: Pose
    object_designator: FrozenObject
    arm: Arms
    grasp: Grasp
    keep_joint_states: bool
    pickup_prepose_distance: float

    # @classproperty
    # def explicit_mapping(cls):
    #     return

# specify custom type mappings
type_mappings ={
    PhysicalObject: StringType()
}


# List of all classes that are self-mapped. ADD NEW DESIGNATORS HERE!
self_mapped_classes = [TaskTreeNode, ActionDescription, MoveTorsoAction, SetGripperAction, ParkArmsAction,
                       NavigateAction, LookAtAction, OpenAction, CloseAction, FaceAtAction]

# List of all classes that are explicitly mapped above. ADD NEW DESIGNATORS HERE!
explicitly_mapped_classes = [DetectAction, GraspingAction, ReleaseAction, GripAction, FrozenObject,
                             ReachToPickUpAction, PickUpAction, PlaceAction, TransportAction, ORMActionNode, ORMMotionNode, ORMResolvedActionNode]