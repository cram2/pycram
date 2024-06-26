from typing_extensions import Optional

from .base import RobotState, Designator, MapperArgsMixin, PoseMixin
from .object_designator import ObjectMixin
from ..datastructures.enums import Arms, GripperState, Grasp
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
    gripper: Mapped[Arms]
    motion: Mapped[GripperState]


class Release(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.Release."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    gripper: Mapped[Arms] = mapped_column(init=False)


class GripAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.GripAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    gripper: Mapped[Arms] = mapped_column(init=False)
    effort: Mapped[float] = mapped_column(init=False)
    # TODO grasped_object


class PickUpAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.PickUpAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]
    grasp: Mapped[Grasp]


class PlaceAction(PoseMixin, ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.PlaceAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]


class TransportAction(PoseMixin, ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.TransportAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]


class LookAtAction(PoseMixin, Action):
    """ORM Class of pycram.designators.action_designator.LookAtAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)


class DetectAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.DetectAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)


class OpenAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.OpenAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]
    # distance: Mapped[float] = mapped_column(init=False)


class CloseAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.CloseAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]


class GraspingAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.GraspingAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]


class FaceAtAction(PoseMixin, Action):
    """ORM Class of pycram.designators.action_designator.FaceAtAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
