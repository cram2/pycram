"""
This module defines a set of ORM classes related to motion designators in the pycram framework.

Each motion designator class has its own table in the database with columns representing its attributes.
The MotionDesignator class is the base class that defines the polymorphic behavior of all other motion designator
classes.
"""


from .base import Base, Position, MapperArgsMixin, PositionMixin, QuaternionMixin
from .object_designator import Object, ObjectMixin
from sqlalchemy.orm import Mapped, mapped_column, MappedAsDataclass, relationship
from sqlalchemy import ForeignKey


class Motion(MappedAsDataclass, Base):
    """
    ORM class of pycram.designators.motion_designator.MotionDesignatorDescription

    :ivar id: (Integer) Auto-incrementing primary key
    :ivar dtype: (String) Polymorphic discriminator
    """

    id: Mapped[int] = mapped_column(autoincrement=True, primary_key=True, init=False)
    dtype: Mapped[str] = mapped_column(init=False)

    __mapper_args__ = {
        "polymorphic_identity": "Motion",
        "polymorphic_on": "dtype",
    }


class MoveMotion(PositionMixin, QuaternionMixin, MapperArgsMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.MoveMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)


class PickUpMotion(ObjectMixin, MapperArgsMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.PickUpMotion

    :ivar arm: (String) Name of the arm used
    :ivar gripper: (String) Name of the gripper used
    :ivar grasp: (String) Type of grasp used
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[str] = mapped_column(init=False)
    gripper: Mapped[str] = mapped_column(init=False)
    grasp: Mapped[str] = mapped_column(init=False)


class PlaceMotion(PositionMixin, QuaternionMixin, ObjectMixin, MapperArgsMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.PlaceMotion

    :ivar arm: (String) Name of the arm used
    :ivar gripper: (String) Name of the gripper used
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[str] = mapped_column(init=False)
    gripper: Mapped[str] = mapped_column(init=False)


class AccessingMotion(MapperArgsMixin, Motion):
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
    arm: Mapped[str] = mapped_column(init=False)
    gripper: Mapped[str] = mapped_column(init=False)
    distance: Mapped[float] = mapped_column(init=False)
    drawer_joint: Mapped[str] = mapped_column(init=False)
    drawer_handle: Mapped[str] = mapped_column(init=False)


class MoveTCPMotion(PositionMixin, QuaternionMixin, MapperArgsMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.MoveTCPMotion

    :ivar arm: String specifying which arm to move the TCP of
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    arm: Mapped[str] = mapped_column(init=False)


class LookingMotion(PositionMixin, QuaternionMixin, ObjectMixin, MapperArgsMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.LookingMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)


class MoveGripperMotion(MapperArgsMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.MoveGripperMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    motion: Mapped[str] = mapped_column(init=False)
    gripper: Mapped[str] = mapped_column(init=False)
    front_facing_axis: Mapped[int] = mapped_column(ForeignKey(f'{Position.__tablename__}.id'), init=False)
    position: Mapped[Position] = relationship(init=False)


class DetectingMotion(MapperArgsMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.DetectingMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    object_type: Mapped[str] = mapped_column(init=False)
    cam_frame: Mapped[str] = mapped_column(init=False)


class WorldStateDetectingMotion(MapperArgsMixin, Motion):
    """
    ORM class of pycram.designators.motion_designator.WorldStateDetectingMotion
    """

    id: Mapped[int] = mapped_column(ForeignKey(f'{Motion.__tablename__}.id'), primary_key=True, init=False)
    object_type: Mapped[str] = mapped_column(init=False)

