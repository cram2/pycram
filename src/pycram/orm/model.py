import inspect
import sys
from dataclasses import dataclass, field
from datetime import datetime
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
from ..datastructures.enums import TaskStatus
from ..datastructures.pose import PoseStamped
from ..designator import ActionDescription
from ..designators import action_designator
from ..failures import PlanFailure
from ..language import LanguageNode, SequentialNode, RepeatNode, TryInOrderNode, ParallelNode, TryAllNode, CodeNode
from ..plan import ActionNode, MotionNode, PlanNode, ResolvedActionNode, DesignatorNode


# ----------------------------------------------------------------------------------------------------------------------
#            Map all Designators, that are not self-mapping, here.
#            By default all classes are self-mapping, so you only need to add the ones where not every attribute is
#            supposed to be mapped or where an attribute is from a type, which is not mapped itself.
#            Specify the columns(attributes) that are supposed to be tracked in the database.
#            One attribute equals one column. Please refer to the ORMatic documentation for more information.
# ----------------------------------------------------------------------------------------------------------------------



@dataclass
class PlanNodeDAO(ORMaticExplicitMapping):
    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classproperty
    def explicit_mapping(cls) -> Type:
        return PlanNode


@dataclass
class DesignatorNodeDAO(ORMaticExplicitMapping):
    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classproperty
    def explicit_mapping(cls) -> Type:
        return DesignatorNode


@dataclass
class ActionNodeDAO(DesignatorNodeDAO, ORMaticExplicitMapping):

    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None
    @classproperty
    def explicit_mapping(cls) -> Type:
        return ActionNode


@dataclass
class MotionNodeDAO(ORMaticExplicitMapping):
    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classproperty
    def explicit_mapping(cls) -> Type:
        return MotionNode


@dataclass
class ResolvedActionNodeDAO(ORMaticExplicitMapping):

    designator_ref: ActionDescription

    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classproperty
    def explicit_mapping(cls) -> Type:
        return ResolvedActionNode

@dataclass
class TryInOrderDAO(ORMaticExplicitMapping):

    @classproperty
    def explicit_mapping(cls) -> Type:
        return TryInOrderNode

@dataclass
class ParallelNodeDAO(ORMaticExplicitMapping):

    @classproperty
    def explicit_mapping(cls):
        return ParallelNode


@dataclass
class TryAllNodeDAO(ORMaticExplicitMapping):

    @classproperty
    def explicit_mapping(cls):
        return TryAllNode

class CodeNodeDAO(ORMaticExplicitMapping):

    @classproperty
    def explicit_mapping(cls):
        return CodeNode


@dataclass
class FrozenObjectDAO(ORMaticExplicitMapping):
    name: str
    concept: PhysicalObject
    pose: Optional[PoseStamped]

    @classproperty
    def explicit_mapping(cls):
        return FrozenObject

