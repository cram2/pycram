from typing import Optional

from .base import Base, Position, Quaternion
import sqlalchemy
import datetime


class ObjectDesignator(Base):
    """ORM class of pycram.designators.object_designator.ObjectDesignator"""
    __tablename__ = "ObjectDesignator"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    type = sqlalchemy.Column(sqlalchemy.types.String)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
        "polymorphic_on": "type",
    }


class LocatedObject(ObjectDesignator):
    """ORM Class of pycram.designators.object_designator.LocatedObject."""

    __tablename__ = "LocatedObject"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    pose = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"), nullable=True)
    reference_frame = sqlalchemy.Column(sqlalchemy.types.String)
    timestamp = sqlalchemy.Column(sqlalchemy.types.DateTime)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }

    def __init__(self, reference_frame: Optional[str] = None, timestamp: Optional[datetime.datetime] = None):
        super().__init__()
        self.reference_frame = reference_frame
        self.timestamp = timestamp
