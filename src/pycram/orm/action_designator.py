from typing import Optional

from .base import Base, Position, Quaternion
import sqlalchemy
from ..enums import Arms


class Action(Base):
    """ORM class of pycram.designators.action_designator.ActionDesignator.
    The purpose of this class is to correctly map the inheritance from the action designator class into the database.
    Inheritance is implemented as Joined Table Inheritance (see https://docs.sqlalchemy.org/en/20/orm/inheritance.html)
    """
    __tablename__ = "Action"
    dtype = sqlalchemy.Column(sqlalchemy.types.String(255))
    robot_state = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("RobotState.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
        "polymorphic_on": "dtype",
    }


class ParkArmsAction(Action):
    """ORM Class of pycram.designators.action_designator.ParkArmsDesignator."""
    __tablename__ = "ParkArms"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    arm = sqlalchemy.Column(sqlalchemy.types.Enum(Arms), nullable=False)

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
    position = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id", ))
    orientation = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Quaternion.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }

    def __init__(self, position: Optional[int] = None, orientation: Optional[int] = None):
        super().__init__()
        self.position = position
        self.orientation = orientation


class MoveTorsoAction(Action):
    """ORM Class of pycram.designators.action_designator.MoveTorsoAction."""
    __tablename__ = "MoveTorso"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    position = sqlalchemy.Column(sqlalchemy.types.Float)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }

    def __init__(self, position: Optional[float] = None):
        super(MoveTorsoAction, self).__init__()
        self.position = position


class SetGripperAction(Action):
    """ORM Class of pycram.designators.action_designator.SetGripperAction."""
    __tablename__ = "SetGripper"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    gripper = sqlalchemy.Column(sqlalchemy.types.String(255), nullable=False)
    motion = sqlalchemy.Column(sqlalchemy.types.String(255), nullable=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }

    def __init__(self, gripper: str, motion: str):
        super(SetGripperAction, self).__init__()
        self.gripper = gripper
        self.motion = motion


class Release(Action):
    """ORM Class of pycram.designators.action_designator.Release."""
    __tablename__ = "Release"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    gripper = sqlalchemy.Column(sqlalchemy.types.String(255), nullable=False)
    object = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class GripAction(Action):
    """ORM Class of pycram.designators.action_designator.GripAction."""
    __tablename__ = "Grip"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    gripper = sqlalchemy.Column(sqlalchemy.types.String(255), nullable=False)
    effort = sqlalchemy.Column(sqlalchemy.types.Float, nullable=False)
    object = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))
    # TODO grasped_object

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class PickUpAction(Action):
    """ORM Class of pycram.designators.action_designator.PickUpAction."""
    __tablename__ = "PickUp"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    arm = sqlalchemy.Column(sqlalchemy.types.String(255))
    grasp = sqlalchemy.Column(sqlalchemy.types.String(255))
    object = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }

    def __init__(self, arm: str, grasp: str):
        super(PickUpAction, self).__init__()
        self.arm = arm
        self.grasp = grasp


class PlaceAction(Action):
    """ORM Class of pycram.designators.action_designator.PlaceAction."""
    __tablename__ = "Place"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    arm = sqlalchemy.Column(sqlalchemy.types.String(255), nullable=False)
    position = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"))
    orientation = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Quaternion.id"))
    object = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }

    def __init__(self, arm: str):
        super(PlaceAction, self).__init__()
        self.arm = arm


class TransportAction(Action):
    """ORM Class of pycram.designators.action_designator.TransportAction."""
    __tablename__ = "Transport"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    arm = sqlalchemy.Column(sqlalchemy.types.String(255), nullable=False)
    position = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"))
    orientation = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Quaternion.id"))
    object = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class LookAtAction(Action):
    """ORM Class of pycram.designators.action_designator.LookAtAction."""
    __tablename__ = "LookAt"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    position = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class DetectAction(Action):
    """ORM Class of pycram.designators.action_designator.DetectAction."""
    __tablename__ = "Detect"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    object = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class OpenAction(Action):
    """ORM Class of pycram.designators.action_designator.OpenAction."""
    __tablename__ = "Open"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    arm = sqlalchemy.Column(sqlalchemy.types.String(255), nullable=False)
    distance = sqlalchemy.Column(sqlalchemy.types.Float, nullable=False)
    object = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class CloseAction(Action):
    """ORM Class of pycram.designators.action_designator.CloseAction."""
    __tablename__ = "Close"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Action.id"), primary_key=True)
    arm = sqlalchemy.Column(sqlalchemy.types.String(255), nullable=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }
