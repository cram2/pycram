"""Implementation of ORM classes associated with pycram.task."""
import sqlalchemy
from sqlalchemy.orm import *
from .base import Base
import datetime


class TaskTreeNode(Base):
    """ORM equivalent of pycram.task.TaskTreeNode."""

    __tablename__ = "TaskTreeNode"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    start_time: Mapped[datetime.datetime]
    end_time: Mapped[datetime.datetime] = mapped_column(nullable=True)
    status: Mapped[str]
    parent: Mapped["TaskTreeNode"] = mapped_column(sqlalchemy.ForeignKey("TaskTreeNode.id",), nullable=True)


    def __init__(self, start_time: datetime.datetime, end_time: datetime.datetime, status, parent):
        super().__init__()
        self.status = status
        self.start_time = start_time
        self.end_time = end_time
        self.parent = parent

    def __repr__(self):
        return f"{self.id}, {self.start_time}, {self.end_time}, {self.status}"
