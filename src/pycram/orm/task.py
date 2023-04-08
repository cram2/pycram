"""Implementation of ORM classes associated with pycram.task."""
import sqlalchemy
from sqlalchemy.orm import *
from .base import Base
import datetime


class TaskTreeNode(Base):
    """ORM equivalent of pycram.task.TaskTreeNode."""

    __tablename__ = "TaskTreeNode"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    code = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Code.id"))
    start_time = sqlalchemy.Column(sqlalchemy.types.DateTime)
    end_time = sqlalchemy.Column(sqlalchemy.types.DateTime, nullable=True)
    status = sqlalchemy.Column(sqlalchemy.types.String)
    parent = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("TaskTreeNode.id"), nullable=True)

    def __init__(self, code: int = None, start_time: datetime.datetime = None, end_time: datetime.datetime = None,
                 status: str = None, parent: int = None):
        super().__init__()
        self.code = code
        self.status = status
        self.start_time = start_time
        self.end_time = end_time
        self.parent = parent


class Code(Base):

    __tablename__ = "Code"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    function = sqlalchemy.Column(sqlalchemy.types.String)

    def __init__(self, function: str = None):
        super().__init__()
        self.function = function
