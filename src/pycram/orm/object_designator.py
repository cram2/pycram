from dataclasses import field
from typing import Optional

from pycram.orm.base import Base, MapperArgsMixin, PoseMixin, Pose
from sqlalchemy.orm import Mapped, mapped_column, declared_attr, relationship, MappedAsDataclass
from sqlalchemy import ForeignKey
from pycram.datastructures.enums import ObjectType


class ObjectMixin(MappedAsDataclass):
    """
    ObjectMixin holds a foreign key column and its relationship to the referenced table.
    For information about Mixins, see https://docs.sqlalchemy.org/en/13/orm/extensions/declarative/mixins.html
    """

    __abstract__ = True
    object_to_init: bool = field(default=False, init=False)

    @declared_attr
    def object_id(self) -> Mapped[int]:
        return mapped_column(ForeignKey(f'{Object.__tablename__}.id'), init=self.object_to_init)

    @declared_attr
    def object(self):
        return relationship(Object.__tablename__, init=False)


class Object(PoseMixin, Base):
    """ORM class of pycram.designators.object_designator.ObjectDesignator"""

    dtype: Mapped[str] = mapped_column(init=False)
    obj_type: Mapped[Optional[ObjectType]]
    name: Mapped[str]

    __mapper_args__ = {
        "polymorphic_identity": "Object",
        "polymorphic_on": "dtype",
    }


class ObjectPart(Object):
    """ORM Class of pycram.designators.object_designator.LocatedObject."""

    id: Mapped[int] = mapped_column(ForeignKey(f'{Object.__tablename__}.id'), primary_key=True, init=False)
    # part_of: Mapped[int] = mapped_column(ForeignKey(f'{Object.__tablename__}.id'), init=False)

    __mapper_args__ = {
        "polymorphic_identity": "ObjectPart",
        "inherit_condition": Object.id == id
    }


class BelieveObject(MapperArgsMixin, Object):

    id: Mapped[int] = mapped_column(ForeignKey(f'{Object.__tablename__}.id'), primary_key=True, init=False)
