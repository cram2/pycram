
from .base import Base, Position, Quaternion, MapperArgsMixin, PositionMixin, QuaternionMixin
from sqlalchemy.orm import Mapped, mapped_column, MappedAsDataclass, relationship
from sqlalchemy import ForeignKey


class Object(PositionMixin, QuaternionMixin, MappedAsDataclass, Base):
    """ORM class of pycram.designators.object_designator.ObjectDesignator"""

    dtype: Mapped[str] = mapped_column(init=False)
    type: Mapped[str]
    name: Mapped[str]

    __mapper_args__ = {
        "polymorphic_identity": "Object",
        "polymorphic_on": "dtype",
    }


class ObjectPart(Object):
    """ORM Class of pycram.designators.object_designator.LocatedObject."""

    id: Mapped[int] = mapped_column(ForeignKey("Object.id"), primary_key=True, init=False)
    part_of: Mapped[int] = mapped_column(ForeignKey("Object.id"), init=False)

    __mapper_args__ = {
        "polymorphic_identity": "ObjectPart",
        "inherit_condition": Object.id == id
    }


class BelieveObject(MapperArgsMixin, Object):

    id: Mapped[int] = mapped_column(ForeignKey("Object.id"), primary_key=True, init=False)
