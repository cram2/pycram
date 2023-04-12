"""
This module defines a set of ORM classes related to motion designators in the pycram framework.

Each motion designator class has its own table in the database with columns representing its attributes.
The MotionDesignator class is the base class that defines the polymorphic behavior of all other motion designator
classes.
"""

import sqlalchemy

from .base import Base


class MotionDesignator(Base):
    """
    ORM class of pycram.designators.motion_designator.MotionDesignatorDescription

    :ivar id: (Integer) Auto-incrementing primary key
    :ivar dtype: (String) Polymorphic discriminator
    """
    __tablename__ = "Motion"

    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    dtype = sqlalchemy.Column(sqlalchemy.types.String)

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
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Motion.id"), primary_key=True)
    position = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"))
    orientation = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Quaternion.id"))
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
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Motion.id"), primary_key=True)
    object = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))
    arm = sqlalchemy.Column(sqlalchemy.types.String)
    gripper = sqlalchemy.Column(sqlalchemy.types.String)
    grasp = sqlalchemy.Column(sqlalchemy.types.String)
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
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Motion.id"), primary_key=True)
    object = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))
    arm = sqlalchemy.Column(sqlalchemy.types.String)
    gripper = sqlalchemy.Column(sqlalchemy.types.String)
    position = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"))
    orientation = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Quaternion.id"))
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
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Motion.id"), primary_key=True)
    part_of = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))
    arm = sqlalchemy.Column(sqlalchemy.types.String)
    gripper = sqlalchemy.Column(sqlalchemy.types.String)
    distance = sqlalchemy.Column(sqlalchemy.types.Float)
    drawer_joint = sqlalchemy.Column(sqlalchemy.types.String)
    drawer_handle = sqlalchemy.Column(sqlalchemy.types.String)

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
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Motion.id"), primary_key=True)
    position = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"))
    orientation = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Quaternion.id"))
    arm = sqlalchemy.Column(sqlalchemy.types.String)

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
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Motion.id"), primary_key=True)
    position = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"))
    orientation = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Quaternion.id"))
    object = sqlalchemy.Column(sqlalchemy.Integer, sqlalchemy.ForeignKey("Object.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class MoveGripperMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.MoveGripperMotion
    """

    __tablename__ = "MoveGripperMotion"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Motion.id"), primary_key=True)
    motion = sqlalchemy.Column(sqlalchemy.types.String)
    gripper = sqlalchemy.Column(sqlalchemy.types.String)
    front_facing_axis = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"))
    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class DetectingMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.DetectingMotion
    """

    __tablename__ = "DetectingMotion"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Motion.id"), primary_key=True)
    object_type = sqlalchemy.Column(sqlalchemy.types.String)
    cam_frame = sqlalchemy.Column(sqlalchemy.types.String)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class WorldStateDetectingMotion(MotionDesignator):
    """
    ORM class of pycram.designators.motion_designator.WorldStateDetectingMotion
    """

    __tablename__ = "WorldStateDetectingMotion"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Motion.id"), primary_key=True)
    object_type = sqlalchemy.Column(sqlalchemy.types.String)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }
