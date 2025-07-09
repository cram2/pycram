import inspect
import sys
from dataclasses import dataclass, field
from datetime import datetime
from typing import Type, List

from ormatic.dao import AlternativeMapping
from random_events.utils import recursive_subclasses
from typing_extensions import Optional, Any, Dict

from pycrap.ontologies import PhysicalObject
from .casts import StringType
from ..datastructures import grasp
from ..datastructures import pose
from ..datastructures.dataclasses import FrozenObject
from ..datastructures.enums import TaskStatus
from ..datastructures.pose import PoseStamped
from ..failures import PlanFailure
from ..language import LanguageNode, RepeatNode, TryInOrderNode, ParallelNode, TryAllNode, CodeNode, \
    MonitorNode
from ..plan import ActionNode, MotionNode, PlanNode, ResolvedActionNode, DesignatorNode
from ..robot_plans import ActionDescription


# ----------------------------------------------------------------------------------------------------------------------
#            Map all Designators, that are not self-mapping, here.
#            By default all classes are self-mapping, so you only need to add the ones where not every attribute is
#            supposed to be mapped or where an attribute is from a type, which is not mapped itself.
#            Specify the columns(attributes) that are supposed to be tracked in the database.
#            One attribute equals one column. Please refer to the ORMatic documentation for more information.
# ----------------------------------------------------------------------------------------------------------------------


@dataclass
class PlanNodeMapping(AlternativeMapping[PlanNode]):
    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classmethod
    def to_dao(cls, obj: PlanNode, memo: Dict[Type, Any] = None):
        """
        Convert a PlanNode to a PlanNodeDAO.
        """
        return cls(
            status=obj.status,
            start_time=obj.start_time,
            end_time=obj.end_time,
            reason=obj.reason
        )


@dataclass
class DesignatorNodeMapping(AlternativeMapping[DesignatorNode]):
    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classmethod
    def to_dao(cls, obj: DesignatorNode, memo: Dict[Type, Any] = None):
        """
        Convert a DesignatorNode to a DesignatorNodeDAO.
        """
        return cls(
            status=obj.status,
            start_time=obj.start_time,
            end_time=obj.end_time,
            reason=obj.reason
        )


@dataclass
class ActionNodeMapping(AlternativeMapping[ActionNode]):

    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None


@dataclass
class MotionNodeMapping(AlternativeMapping[MotionNode]):
    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classmethod
    def to_dao(cls, obj: MotionNode, memo: Dict[Type, Any] = None):
        """
        Convert a MotionNode to a MotionNodeDAO.
        """
        return cls(
            status=obj.status,
            start_time=obj.start_time,
            end_time=obj.end_time,
            reason=obj.reason
        )

    @classmethod
    def to_dao(cls, obj: ActionNode, memo: Dict[Type, Any] = None):
        """
        Convert an ActionNode to an ActionNodeDAO.
        """
        return cls(
            status=obj.status,
            start_time=obj.start_time,
            end_time=obj.end_time,
            reason=obj.reason
        )

@dataclass
class ResolvedActionNodeMapping(AlternativeMapping[ResolvedActionNode]):

    designator_ref: ActionDescription

    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classmethod
    def to_dao(cls, obj: ResolvedActionNode, memo: Dict[Type, Any] = None):
        """
        Convert a ResolvedActionNode to a ResolvedActionNodeDAO.
        """
        return cls(
            designator_ref=obj.designator_ref,
            status=obj.status,
            start_time=obj.start_time,
            end_time=obj.end_time,
            reason=obj.reason
        )

@dataclass
class TryInOrderMapping(AlternativeMapping[TryInOrderNode]):

    @classmethod
    def to_dao(cls, obj: TryInOrderNode, memo: Dict[Type, Any] = None):
        """
        Convert a TryInOrderNode to a TryInOrderNodeDAO.
        """
        return cls()

@dataclass
class ParallelNodeMapping(AlternativeMapping[ParallelNode]):

    @classmethod
    def to_dao(cls, obj: ParallelNode, memo: Dict[Type, Any] = None):
        """
        Convert a ParallelNode to a ParallelNodeDAO.
        """
        return cls()


@dataclass
class TryAllNodeMapping(AlternativeMapping[TryAllNode]):

    @classmethod
    def to_dao(cls, obj: TryAllNode, memo: Dict[Type, Any] = None):
        """
        Convert a TryAllNode to a TryAllNodeDAO.
        """
        return cls()


@dataclass
class CodeNodeMapping(AlternativeMapping[CodeNode]):

    @classmethod
    def to_dao(cls, obj: CodeNode, memo: Dict[Type, Any] = None):
        """
        Convert a CodeNode to a CodeNodeDAO.
        """
        return cls(
        )


@dataclass
class FrozenObjectMapping(AlternativeMapping[FrozenObject]):
    name: str
    concept: Type[PhysicalObject]
    pose: Optional[PoseStamped]

    @classmethod
    def to_dao(cls, obj: FrozenObject, memo: Dict[Type, Any] = None):
        """
        Convert a FrozenObject to a FrozenObjectDAO.
        """
        return cls(
            name=obj.name,
            concept=obj.concept,
            pose=obj.pose
        )


@dataclass
class MonitorNodeMapping(AlternativeMapping[MonitorNode]):

    @classmethod
    def to_dao(cls, obj: MonitorNode, memo: Dict[Type, Any] = None):
        """
        Convert a MonitorNode to a MonitorNodeDAO.
        """
        return cls()

