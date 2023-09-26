"""Implementation of ORM classes associated with pycram.task."""
from typing import Optional

from sqlalchemy.types import String
from sqlalchemy import ForeignKey
from sqlalchemy.orm import MappedAsDataclass, Mapped, mapped_column

from .base import Base
from ..enums import TaskStatus
import datetime


class TaskTreeNode(Base):
    """ORM equivalent of pycram.task.TaskTreeNode."""

    __tablename__ = "TaskTreeNode"

    code: Mapped[int] = mapped_column(ForeignKey("Code.id"))
    start_time: Mapped[datetime.datetime]
    end_time: Mapped[datetime.datetime] = mapped_column(nullable=True)
    status: Mapped[TaskStatus]
    reason: Mapped[str] = mapped_column(String(255), nullable=True)
    parent: Mapped[int] = mapped_column(ForeignKey("TaskTreeNode.id"), nullable=True)

    # def __init__(self, code: int = None, start_time: datetime.datetime = None, end_time: datetime.datetime = None,
    #              status: str = None, reason: Optional[str] = None, parent: int = None):
    #     super().__init__()
    #     self.code = code
    #     self.status = status
    #     self.start_time = start_time
    #     self.end_time = end_time
    #     self.reason = reason
    #     self.parent = parent


class Code(Base):
    """ORM equivalent of pycram.task.Code."""

    __tablename__ = "Code"

    function: Mapped[str] = mapped_column(String(255))
    designator: Mapped[int] = mapped_column(ForeignKey("Action.id"), nullable=True)

    # def __init__(self, function: str = None, designator: Optional[int] = None):
    #     super().__init__()
    #     self.function = function
    #     self.designator = designator
