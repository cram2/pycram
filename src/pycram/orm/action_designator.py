from typing import Optional

from .base import Base, Position, Quaternion
import sqlalchemy


class Action(Base):
    """ORM class of pycram.designators.action_designator.ActionDesignator.
    The purpose of this class is to correctly map the inheritance from the action designator class into the database.
    Inheritance is implemented as Joined Table Inheritance (see https://docs.sqlalchemy.org/en/20/orm/inheritance.html)
    """
    __tablename__ = "Action"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    type = sqlalchemy.Column(sqlalchemy.types.String)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
        "polymorphic_on": "type",
    }


class ParkArmsAction(Action):
    """ORM Class of pycram.designators.action_designator.ParkArmsDesignator."""
    __tablename__ = "ParkArms"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    arm = sqlalchemy.Column(sqlalchemy.types.String, nullable=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }

    def __init__(self, arm: str = None):
        super().__init__()
        self.arm = arm


class NavigateAction(Action):
    """ORM Class of pycram.designators.action_designator.NavigateAction."""

    __tablename__ = "Navigate"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    position = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"))
    orientation = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Quaternion.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }

    def __init__(self, position: Optional[int] = None, orientation: Optional[int] = None):
        super().__init__()
        self.position = position
        self.orientation = orientation
