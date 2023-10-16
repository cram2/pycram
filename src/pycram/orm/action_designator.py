from typing import Optional

from .base import RobotState, Position, Quaternion, Base, MapperArgsMixin, ObjectMixin, PositionMixin, QuaternionMixin
from ..enums import Arms
from sqlalchemy.orm import Mapped, mapped_column, MappedAsDataclass, relationship
from sqlalchemy import ForeignKey


class Action(MappedAsDataclass, Base):
    """ORM class of pycram.designators.action_designator.ActionDesignator.
    The purpose of this class is to correctly map the inheritance from the action designator class into the database.
    Inheritance is implemented as Joined Table Inheritance (see https://docs.sqlalchemy.org/en/20/orm/inheritance.html)
    """

    dtype: Mapped[str] = mapped_column(init=False)
    robot_state: Mapped[int] = mapped_column(ForeignKey("RobotState.id"), init=False)
    robot_state_table_entry: Mapped[RobotState] = relationship(init=False)

    __mapper_args__ = {
        "polymorphic_identity": "Action",
        "polymorphic_on": "dtype",
    }


class ParkArmsAction(MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.ParkArmsDesignator."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    arm: Mapped[Arms] = mapped_column(default=None)


class NavigateAction(MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.NavigateAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    position: Mapped[int] = mapped_column(ForeignKey("Position.id", ), default=None)
    position_table_entry: Mapped[Position] = relationship(init=False)
    orientation: Mapped[int] = mapped_column(ForeignKey("Quaternion.id"), default=None)
    orientation_table_entry: Mapped[Quaternion] = relationship(init=False)


class MoveTorsoAction(MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.MoveTorsoAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    position: Mapped[Optional[float]] = mapped_column(default=None)


class SetGripperAction(MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.SetGripperAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    gripper: Mapped[str]
    motion: Mapped[str]


class Release(ObjectMixin, MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.Release."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    gripper: Mapped[str] = mapped_column(init=False)


class GripAction(ObjectMixin, MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.GripAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    gripper: Mapped[str] = mapped_column(init=False)
    effort: Mapped[float] = mapped_column(init=False)
    # TODO grasped_object


class PickUpAction(ObjectMixin, MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.PickUpAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    arm: Mapped[str]
    grasp: Mapped[str]


class PlaceAction(PositionMixin, QuaternionMixin, ObjectMixin, MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.PlaceAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    arm: Mapped[str]


class TransportAction(PositionMixin, QuaternionMixin, ObjectMixin, MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.TransportAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    arm: Mapped[str] = mapped_column(init=False)


class LookAtAction(PositionMixin, MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.LookAtAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)


class DetectAction(ObjectMixin, MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.DetectAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)


class OpenAction(ObjectMixin, MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.OpenAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    arm: Mapped[str] = mapped_column(init=False)
    distance: Mapped[float] = mapped_column(init=False)


class CloseAction(MapperArgsMixin, Action):
    """ORM Class of pycram.designators.action_designator.CloseAction."""

    id: Mapped[int] = mapped_column(ForeignKey("Action.id"), primary_key=True, init=False)
    arm: Mapped[str] = mapped_column(init=False)

