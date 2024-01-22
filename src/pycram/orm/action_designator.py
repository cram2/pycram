from typing import Optional

from .base import RobotState, Designator, MapperArgsMixin, PoseMixin
from .object_designator import ObjectMixin, Object
from ..enums import Arms
from sqlalchemy.orm import Mapped, mapped_column, relationship
from sqlalchemy import ForeignKey


class Action(MapperArgsMixin, Designator):
    """ORM class of pycram.designators.action_designator.ActionDesignator.
    The purpose of this class is to correctly map the inheritance from the action designator class into the database.
    Inheritance is implemented as Joined Table Inheritance (see https://docs.sqlalchemy.org/en/20/orm/inheritance.html)
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Designator.__tablename__}.id'), primary_key=True, init=False)
    dtype: Mapped[str] = mapped_column("action_dtype", init=False)
    robot_state_id: Mapped[int] = mapped_column(ForeignKey(f"{RobotState.__tablename__}.id"), init=False)
    robot_state: Mapped[RobotState] = relationship(init=False)


class ParkArmsAction(Action):
    """ORM Class of pycram.designators.action_designator.ParkArmsDesignator."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms] = mapped_column(default=None)


class NavigateAction(PoseMixin, Action):
    """ORM Class of pycram.designators.action_designator.NavigateAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)


class MoveTorsoAction(Action):
    """ORM Class of pycram.designators.action_designator.MoveTorsoAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    position: Mapped[Optional[float]] = mapped_column(default=None)


class SetGripperAction(Action):
    """ORM Class of pycram.designators.action_designator.SetGripperAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    gripper: Mapped[str]
    motion: Mapped[str]


class Release(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.Release."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    gripper: Mapped[str] = mapped_column(init=False)


class GripAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.GripAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    gripper: Mapped[str] = mapped_column(init=False)
    effort: Mapped[float] = mapped_column(init=False)
    # TODO grasped_object


class PickUpAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.PickUpAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[str]
    grasp: Mapped[str]


class PlaceAction(PoseMixin, ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.PlaceAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[str]


class TransportAction(PoseMixin, ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.TransportAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[str]


class LookAtAction(PoseMixin, Action):
    """ORM Class of pycram.designators.action_designator.LookAtAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)


class DetectAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.DetectAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)


class OpenAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.OpenAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[str]
    # distance: Mapped[float] = mapped_column(init=False)


class CloseAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.CloseAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[str]


class GraspingAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.GraspingAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[str]


class CuttingAction(Action):
    """ORM Class of pycram.designators.action_designator.GraspingAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[str]
    technique: Mapped[Optional[str]]
    slice_thickness: Mapped[Optional[float]]

    object_to_be_cut_id: Mapped[Optional[int]] = mapped_column(ForeignKey(f"{Object.__tablename__}.id"), init=False)
    object_to_be_cut: Object = relationship(Object.__tablename__, init=False, foreign_keys=[object_to_be_cut_id])

    tool_id: Mapped[Optional[int]] = mapped_column(ForeignKey(f"{Object.__tablename__}.id"), init=False)
    tool: Object = relationship(Object.__tablename__, init=False, foreign_keys=[tool_id])


