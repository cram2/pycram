"""
This module defines a set of ORM classes related to motion designators in the pycram framework.

Each motion designator class has its own table in the database with columns representing its attributes.
The MotionDesignator class is the base class that defines the polymorphic behavior of all other motion designator
classes.
"""

import sqlalchemy

from .base import Base
from sqlalchemy.orm import Mapped, mapped_column
from sqlalchemy import String, ForeignKey


class MotionDesignator(Base):
    """
    ORM class of pycram.designators.motion_designator.MotionDesignatorDescription

    :ivar id: (Integer) Auto-incrementing primary key
    :ivar dtype: (String) Polymorphic discriminator
    """
    __tablename__ = "Motion"

    id: Mapped[int] = mapped_column(autoincrement=True, primary_key=True, init=False)
    dtype: Mapped[str] = mapped_column(String(255), init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
        "polymorphic_on": "dtype",
    }


class MoveMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.MoveMotion

    :ivar position: (Integer) Foreign key to Position table
    :ivar orientation: (Integer) Foreign key to Quaternion table
    """
    __tablename__ = "MoveMotion"

    id: Mapped[int] = mapped_column(ForeignKey("Motion.id"), primary_key=True, init=False)
    position: Mapped[int] = mapped_column(ForeignKey("Position.id"), init=False)
    orientation: Mapped[int] = mapped_column(ForeignKey("Quaternion.id"), init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class PickUpMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.PickUpMotion

    :ivar object: (Integer) Foreign key to Object table
    :ivar arm: (String) Name of the arm used
    :ivar gripper: (String) Name of the gripper used
    :ivar grasp: (String) Type of grasp used
    """
    __tablename__ = "PickUpMotion"

    id: Mapped[int] = mapped_column(ForeignKey("Motion.id"), primary_key=True, init=False)
    object: Mapped[int] = mapped_column(ForeignKey("Object.id"), init=False)
    arm: Mapped[str] = mapped_column(String(255), init=False)
    gripper: Mapped[str] = mapped_column(String(255), init=False)
    grasp: Mapped[str] = mapped_column(String(255), init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class PlaceMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.PlaceMotion

    :ivar object: (Integer) Foreign key to Object table
    :ivar arm: (String) Name of the arm used
    :ivar gripper: (String) Name of the gripper used
    :ivar position: (Integer) Foreign key to Position table
    :ivar orientation: (Integer) Foreign key to Quaternion table
    """
    __tablename__ = "PlaceMotion"

    id: Mapped[int] = mapped_column(ForeignKey("Motion.id"), primary_key=True, init=False)
    object: Mapped[int] = mapped_column(ForeignKey("Object.id"), init=False)
    arm: Mapped[str] = mapped_column(String(255), init=False)
    gripper: Mapped[str] = mapped_column(String(255), init=False)
    position: Mapped[int] = mapped_column(ForeignKey("Position.id"), init=False)
    orientation: Mapped[int] = mapped_column(ForeignKey("Quaternion.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class AccessingMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.AccessingMotion

    :ivar part_of: (Integer) Foreign key to Object table
    :ivar arm: (String) Name of the arm used
    :ivar gripper: (String) Name of the gripper used
    :ivar distance: (Float) Distance from the drawer to the robot
    :ivar drawer_joint:
    """
    __tablename__ = "AccessingMotion"

    id: Mapped[int] = mapped_column(ForeignKey("Motion.id"), primary_key=True, init=False)
    part_of: Mapped[int] = mapped_column(ForeignKey("Object.id"), init=False)
    arm: Mapped[str] = mapped_column(String(255), init=False)
    gripper: Mapped[str] = mapped_column(String(255), init=False)
    distance: Mapped[float] = mapped_column(init=False)
    drawer_joint: Mapped[str] = mapped_column(String(255), init=False)
    drawer_handle: Mapped[str] = mapped_column(String(255), init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class MoveTCPMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.MoveTCPMotion

    :ivar position: Integer id of the position to move the TCP to
    :ivar orientation: Integer id of the quaternion orientation to move the TCP to
    :ivar arm: String specifying which arm to move the TCP of
    """
    __tablename__ = "MoveTCPMotion"

    id: Mapped[int] = mapped_column(ForeignKey("Motion.id"), primary_key=True, init=False)
    position: Mapped[int] = mapped_column(ForeignKey("Position.id"), init=False)
    orientation: Mapped[int] = mapped_column(ForeignKey("Quaternion.id"), init=False)
    arm: Mapped[str] = mapped_column(String(255), init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class LookingMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.LookingMotion

    :ivar position: Integer id of the position to move the TCP to
    :ivar orientation: Integer id of the quaternion orientation to move the TCP to
    :ivar object: (Integer) Foreign key to Object table
    """

    __tablename__ = "LookingMotion"

    id: Mapped[int] = mapped_column(ForeignKey("Motion.id"), primary_key=True, init=False)
    position: Mapped[int] = mapped_column(ForeignKey("Position.id"), init=False)
    orientation: Mapped[int] = mapped_column(ForeignKey("Quaternion.id"), init=False)
    object: Mapped[int] = mapped_column(ForeignKey("Object.id"), init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class MoveGripperMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.MoveGripperMotion
    """

    __tablename__ = "MoveGripperMotion"

    id: Mapped[int] = mapped_column(ForeignKey("Motion.id"), primary_key=True, init=False)
    motion: Mapped[str] = mapped_column(String(255), init=False)
    gripper: Mapped[str] = mapped_column(String(255), init=False)
    front_facing_axis: Mapped[int] = mapped_column(ForeignKey("Position.id"), init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class DetectingMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.DetectingMotion
    """

    __tablename__ = "DetectingMotion"

    id: Mapped[int] = mapped_column(ForeignKey("Motion.id"), primary_key=True, init=False)
    object_type: Mapped[str] = mapped_column(String(255), init=False)
    cam_frame: Mapped[str] = mapped_column(String(255), init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class WorldStateDetectingMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.WorldStateDetectingMotion
    """

    __tablename__ = "WorldStateDetectingMotion"

    id: Mapped[int] = mapped_column(ForeignKey("Motion.id"), primary_key=True, init=False)
    object_type: Mapped[str] = mapped_column(String(255), init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }
