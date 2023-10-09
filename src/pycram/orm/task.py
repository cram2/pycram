"""Implementation of ORM classes associated with pycram.task."""
from typing import Optional
from sqlalchemy import ForeignKey
from sqlalchemy.orm import MappedAsDataclass, Mapped, mapped_column, relationship
from .action_designator import Action
from .base import Base
from ..enums import TaskStatus
import datetime


class TaskTreeNode(MappedAsDataclass, Base):
    """ORM equivalent of pycram.task.TaskTreeNode."""

    __tablename__ = "TaskTreeNode"

    id: Mapped[int] = mapped_column(autoincrement=True, primary_key=True, init=False)

    code: Mapped[int] = mapped_column(ForeignKey("Code.id"), default=None)
    code_table_entry: Mapped["Code"] = relationship(init=False)
    start_time: Mapped[datetime.datetime] = mapped_column(default=None)
    end_time: Mapped[Optional[datetime.datetime]] = mapped_column(default=None)
    status: Mapped[TaskStatus] = mapped_column(default=None)
    reason: Mapped[Optional[str]] = mapped_column(default=None)
    parent: Mapped[Optional[int]] = mapped_column(ForeignKey("TaskTreeNode.id"), default=None)
    parent_table_entry: Mapped["TaskTreeNode"] = relationship(foreign_keys=[parent], init=False, remote_side=[id])

    # def __init__(self, code: int = None, start_time: datetime.datetime = None, end_time: datetime.datetime = None,
    #              status: str = None, reason: Optional[str] = None, parent: int = None):
    #     super().__init__()
    #     self.code = code
    #     self.status = status
    #     self.start_time = start_time
    #     self.end_time = end_time
    #     self.reason = reason
    #     self.parent = parent


class Code(MappedAsDataclass, Base):
    """ORM equivalent of pycram.task.Code."""

    __tablename__ = "Code"

    function: Mapped[str] = mapped_column(default=None)
    designator: Mapped[Optional[int]] = mapped_column(ForeignKey("Action.id"), default=None)
    designator_table_entry: Mapped[Action] = relationship(init=False)

    # def __init__(self, function: str = None, designator: Optional[int] = None):
    #     super().__init__()
    #     self.function = function
    #     self.designator = designator
