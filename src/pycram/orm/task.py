"""Implementation of ORM classes associated with pycram.task."""
from typing import Optional

import sqlalchemy
from .base import Base
from ..enums import TaskStatus
import datetime


class TaskTreeNode(Base):
    """ORM equivalent of pycram.task.TaskTreeNode."""

    __tablename__ = "TaskTreeNode"
    code = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Code.id"))
    start_time = sqlalchemy.Column(sqlalchemy.types.DateTime)
    end_time = sqlalchemy.Column(sqlalchemy.types.DateTime, nullable=True)
    status = sqlalchemy.Column(sqlalchemy.types.Enum(TaskStatus))
    reason = sqlalchemy.Column(sqlalchemy.types.String(255), nullable=True)
    parent = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("TaskTreeNode.id"), nullable=True)

    def __init__(self, code: int = None, start_time: datetime.datetime = None, end_time: datetime.datetime = None,
                 status: str = None, reason: Optional[str] = None, parent: int = None):
        super().__init__()
        self.code = code
        self.status = status
        self.start_time = start_time
        self.end_time = end_time
        self.reason = reason
        self.parent = parent


class Code(Base):
    """ORM equivalent of pycram.task.Code."""

    __tablename__ = "Code"
    function = sqlalchemy.Column(sqlalchemy.types.String(255))
    designator = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), nullable=True)

    def __init__(self, function: str = None, designator: Optional[int] = None):
        super().__init__()
        self.function = function
        self.designator = designator
