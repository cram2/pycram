from typing import Optional

from .base import Base, Position, Quaternion
import sqlalchemy
import datetime


class ObjectDesignator(Base):
    """ORM class of pycram.designators.object_designator.ObjectDesignator"""
    __tablename__ = "Object"

    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    dtype = sqlalchemy.Column(sqlalchemy.types.String)
    type = sqlalchemy.Column(sqlalchemy.types.String)
    name = sqlalchemy.Column(sqlalchemy.types.String)
    position = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Position.id"))
    orientation = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Quaternion.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
        "polymorphic_on": "dtype",
    }


class ObjectPart(ObjectDesignator):
    """ORM Class of pycram.designators.object_designator.LocatedObject."""

    __tablename__ = "ObjectPart"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Object.id"), primary_key=True)
    part_of = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Object.id"))

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }


class BelieveObject(ObjectDesignator):
    __tablename__ = "BelieveObject"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, sqlalchemy.ForeignKey("Object.id"), primary_key=True)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }
