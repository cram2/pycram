from typing_extensions import Optional, Dict

from .base import RobotState, Designator, MapperArgsMixin, PoseMixin, GraspMixin
from .object_designator import ObjectMixin
from ..datastructures.enums import Arms, GripperState, DetectionTechnique, DetectionState, TorsoState
from sqlalchemy.orm import Mapped, mapped_column, relationship
from sqlalchemy import ForeignKey, JSON


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
    keep_joint_states: Mapped[bool]


class MoveTorsoAction(Action):
    """ORM Class of pycram.designators.action_designator.MoveTorsoAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    torso_state: Mapped[TorsoState] = mapped_column(default=None)


class SetGripperAction(Action):
    """ORM Class of pycram.designators.action_designator.SetGripperAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    gripper: Mapped[Arms]
    motion: Mapped[GripperState]


class ReleaseAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.Release."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    gripper: Mapped[Arms] = mapped_column(init=False)


class GripAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.GripAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    gripper: Mapped[Arms] = mapped_column(init=False)
    effort: Mapped[float] = mapped_column(init=False)
    # TODO grasped_object


class ReachToPickUpAction(ObjectMixin, GraspMixin, Action):
    """ORM Class of pycram.designators.action_designator.ReachToPickUpAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]
    prepose_distance: Mapped[float]


class PickUpAction(ObjectMixin, GraspMixin, Action):
    """ORM Class of pycram.designators.action_designator.PickUpAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]
    prepose_distance: Mapped[float]


class PlaceAction(PoseMixin, ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.PlaceAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]


class TransportAction(PoseMixin, ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.TransportAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]
    pickup_prepose_distance: Mapped[float]


class LookAtAction(PoseMixin, Action):
    """ORM Class of pycram.designators.action_designator.LookAtAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)


class DetectAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.DetectAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    technique: Mapped[DetectionTechnique] = mapped_column(init=False)
    state: Mapped[DetectionState] = mapped_column(init=False)
    region: Mapped[str] = mapped_column(init=False)

class OpenAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.OpenAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]
    grasping_prepose_distance: Mapped[float]


class CloseAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.CloseAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]
    grasping_prepose_distance: Mapped[float]


class GraspingAction(ObjectMixin, Action):
    """ORM Class of pycram.designators.action_designator.GraspingAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]
    prepose_distance: Mapped[float]


class FaceAtAction(PoseMixin, Action):
    """ORM Class of pycram.designators.action_designator.FaceAtAction."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    keep_joint_states: Mapped[bool]
