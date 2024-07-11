"""
This module defines a set of ORM classes related to motion designators in the pycram framework.

Each motion designator class has its own table in the database with columns representing its attributes.
The MotionDesignator class is the base class that defines the polymorphic behavior of all other motion designator
classes.
"""
from typing_extensions import Optional

from .base import MapperArgsMixin, Designator, PoseMixin
from .object_designator import Object
from sqlalchemy.orm import Mapped, mapped_column, relationship
from sqlalchemy import ForeignKey

from ..datastructures.enums import ObjectType, Arms, GripperState


class Motion(MapperArgsMixin, Designator):
    """
    ORM class of pycram.designators.motion_designator.MotionDesignatorDescription

    :ivar id: (Integer) Auto-incrementing primary key
    :ivar dtype: (String) Polymorphic discriminator
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Designator.__tablename__}.id'), primary_key=True, init=False)
    dtype: Mapped[str] = mapped_column("motion_dtype", init=False)


class MoveMotion(PoseMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.MoveMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)


class AccessingMotion(Motion):
    """
    ORM class of pycram.designators.motion_designator.AccessingMotion

    :ivar arm: (String) Name of the arm used
    :ivar gripper: (String) Name of the gripper used
    :ivar distance: (Float) Distance from the drawer to the robot
    :ivar drawer_joint:
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    part_of: Mapped[int] = mapped_column(ForeignKey(f'{Object.__tablename__}.id'), init=False)
    object: Mapped[Object] = relationship(init=False)
    arm: Mapped[Arms] = mapped_column(init=False)
    gripper: Mapped[str] = mapped_column(init=False)
    distance: Mapped[float] = mapped_column(init=False)
    drawer_joint: Mapped[str] = mapped_column(init=False)
    drawer_handle: Mapped[str] = mapped_column(init=False)


class MoveTCPMotion(PoseMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.MoveTCPMotion

    :ivar arm: String specifying which arm to move the TCP of
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]
    allow_gripper_collision: Mapped[Optional[bool]]


class LookingMotion(PoseMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.LookingMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)


class MoveGripperMotion(Motion):
    """
    ORM class of pycram.designators.motion_designator.MoveGripperMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    motion: Mapped[GripperState]
    gripper: Mapped[Arms]
    allow_gripper_collision: Mapped[Optional[bool]]


class DetectingMotion(Motion):
    """
    ORM class of pycram.designators.motion_designator.DetectingMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    object_type: Mapped[ObjectType]


class WorldStateDetectingMotion(Motion):
    """
    ORM class of pycram.designators.motion_designator.WorldStateDetectingMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    object_type: Mapped[str] = mapped_column(init=False)


class OpeningMotion(Motion):
    """
    ORM class of pycram.designators.motion_designator.OpeningMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]


class ClosingMotion(Motion):
    """
    ORM class of pycram.designators.motion_designator.ClosingMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[Arms]
