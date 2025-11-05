from dataclasses import dataclass, field
from datetime import datetime
from typing import Type

import numpy as np
from ormatic.dao import AlternativeMapping
from sqlalchemy import TypeDecorator, types
from typing_extensions import Optional

from ..datastructures.enums import TaskStatus
from ..failures import PlanFailure
from ..language import TryInOrderNode, ParallelNode, TryAllNode, CodeNode, \
    MonitorNode
from ..plan import ActionNode, MotionNode, PlanNode, ResolvedActionNode, DesignatorNode
from ..robot_plans import ActionDescription, BaseMotion


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
    def create_instance(cls, obj: PlanNode):
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
    def create_instance(cls, obj: DesignatorNode):
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
    action: Type[ActionNode]
    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classmethod
    def create_instance(cls, obj: ActionNode):
        """
        Convert an ActionNode to an ActionNodeDAO.
        """
        return cls(
            status=obj.status,
            action=obj.action,
            start_time=obj.start_time,
            end_time=obj.end_time,
            reason=obj.reason
        )


@dataclass
class MotionNodeMapping(AlternativeMapping[MotionNode]):
    action: Type[BaseMotion]
    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classmethod
    def create_instance(cls, obj: MotionNode):
        """
        Convert a MotionNode to a MotionNodeDAO.
        """
        return cls(
            status=obj.status,
            action=obj.action,
            start_time=obj.start_time,
            end_time=obj.end_time,
            reason=obj.reason
        )


@dataclass
class ResolvedActionNodeMapping(AlternativeMapping[ResolvedActionNode]):
    designator_ref: ActionDescription
    action: Type[ActionDescription]

    status: TaskStatus = TaskStatus.CREATED
    start_time: Optional[datetime] = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None
    reason: Optional[PlanFailure] = None

    @classmethod
    def create_instance(cls, obj: ResolvedActionNode):
        """
        Convert a ResolvedActionNode to a ResolvedActionNodeDAO.
        """
        return cls(
            designator_ref=obj.designator_ref,
            action=obj.action,
            status=obj.status,
            start_time=obj.start_time,
            end_time=obj.end_time,
            reason=obj.reason
        )


@dataclass
class TryInOrderMapping(AlternativeMapping[TryInOrderNode]):

    @classmethod
    def create_instance(cls, obj: TryInOrderNode):
        """
        Convert a TryInOrderNode to a TryInOrderNodeDAO.
        """
        return cls()


@dataclass
class ParallelNodeMapping(AlternativeMapping[ParallelNode]):

    @classmethod
    def create_instance(cls, obj: ParallelNode):
        """
        Convert a ParallelNode to a ParallelNodeDAO.
        """
        return cls()


@dataclass
class TryAllNodeMapping(AlternativeMapping[TryAllNode]):

    @classmethod
    def create_instance(cls, obj: TryAllNode):
        """
        Convert a TryAllNode to a TryAllNodeDAO.
        """
        return cls()


@dataclass
class CodeNodeMapping(AlternativeMapping[CodeNode]):

    @classmethod
    def create_instance(cls, obj: CodeNode):
        """
        Convert a CodeNode to a CodeNodeDAO.
        """
        return cls(
        )


@dataclass
class MonitorNodeMapping(AlternativeMapping[MonitorNode]):

    @classmethod
    def create_instance(cls, obj: MonitorNode):
        """
        Convert a MonitorNode to a MonitorNodeDAO.
        """
        return cls()


#
# @dataclass
# class MonitorNodeDAO(ORMaticExplicitMapping):
#
#     @classproperty
#     def explicit_mapping(cls) -> Type:
#         return MonitorNode


class NumpyType(TypeDecorator):
    """
    Type that casts field which are of numpy nd array type
    """

    impl = types.LargeBinary(4 * 1024 * 1024 * 1024 - 1)  # 4 GB max

    def process_bind_param(self, value: np.ndarray, dialect):
        array = value.astype(np.float64)
        return array.tobytes()

    def process_result_value(self, value: impl, dialect) -> Optional[np.ndarray]:
        return np.frombuffer(value, dtype=np.float64)
