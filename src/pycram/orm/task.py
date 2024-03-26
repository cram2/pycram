"""Implementation of ORM classes associated with pycram.task."""
from typing import Optional
from sqlalchemy import ForeignKey
from sqlalchemy.orm import MappedAsDataclass, Mapped, mapped_column, relationship
from .action_designator import Action
from .base import Base, Designator
from .motion_designator import Motion
from ..enums import TaskStatus
import datetime


class TaskTreeNode(Base):
    """ORM equivalent of pycram.task.TaskTreeNode."""

    id: Mapped[int] = mapped_column(autoincrement=True, primary_key=True, init=False)
    """id overriden in order to be able to set the remote_side of the parent attribute"""

    code_id: Mapped[int] = mapped_column(ForeignKey("Code.id"), init=False)
    code: Mapped["Code"] = relationship(init=False)
    start_time: Mapped[datetime.datetime]
    end_time: Mapped[Optional[datetime.datetime]]
    status: Mapped[TaskStatus]
    reason: Mapped[Optional[str]]
    parent_id: Mapped[Optional[int]] = mapped_column(ForeignKey("TaskTreeNode.id"), init=False)
    parent: Mapped[Optional["TaskTreeNode"]] = relationship(init=False, remote_side=[id])


class Code(Base):
    """ORM equivalent of pycram.task.Code."""

    function: Mapped[str] = mapped_column(default=None)
    designator_id: Mapped[Optional[int]] = mapped_column(ForeignKey(f'{Designator.__tablename__}.id'), default=None)
    designator: Mapped[Designator] = relationship(init=False)

