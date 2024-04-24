"""Implementation of ORM classes associated with pycram.task."""
from typing_extensions import Optional
from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship
from .base import Base, Designator
from pycram.datastructures.enums import TaskStatus
import datetime


class TaskTreeNode(Base):
    """ORM equivalent of pycram.task.TaskTreeNode."""

    id: Mapped[int] = mapped_column(autoincrement=True, primary_key=True, init=False)

    action_id: Mapped[Optional[int]] = mapped_column(ForeignKey(f'{Designator.__tablename__}.id'), init=False)
    action: Mapped[Optional[Designator]] = relationship(init=False)

    start_time: Mapped[datetime.datetime]
    end_time: Mapped[Optional[datetime.datetime]]

    status: Mapped[TaskStatus]
    reason: Mapped[Optional[str]]

    parent_id: Mapped[Optional[int]] = mapped_column(ForeignKey("TaskTreeNode.id"), init=False)
    parent: Mapped[Optional["TaskTreeNode"]] = relationship(init=False, remote_side=[id])


