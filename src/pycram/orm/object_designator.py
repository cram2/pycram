
from .base import Base, Position, Quaternion
from sqlalchemy.orm import Mapped, mapped_column, MappedAsDataclass, relationship
from sqlalchemy import ForeignKey


class ObjectDesignator(MappedAsDataclass, Base):
    """ORM class of pycram.designators.object_designator.ObjectDesignator"""

    __tablename__ = "Object"

    id: Mapped[int] = mapped_column(autoincrement=True, primary_key=True, init=False)
    dtype: Mapped[str] = mapped_column(init=False)
    type: Mapped[str]
    name: Mapped[str]
    position: Mapped[int] = mapped_column(ForeignKey("Position.id"), init=False)
    position_table_entry: Mapped[Position] = relationship(init=False)
    orientation: Mapped[int] = mapped_column(ForeignKey("Quaternion.id"), init=False)
    orientation_table_entry: Mapped[Quaternion] = relationship(init=False)

    # def __init__(self, type: str, name: str):
    #     super().__init__()
    #     self.type = type
    #     self.name = name

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
        "polymorphic_on": "dtype",
    }


class ObjectPart(ObjectDesignator):
    """ORM Class of pycram.designators.object_designator.LocatedObject."""

    __tablename__ = "ObjectPart"

    id: Mapped[int] = mapped_column(ForeignKey("Object.id"), primary_key=True, init=False)
    part_of: Mapped[int] = mapped_column(ForeignKey("Object.id"), init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
        "inherit_condition": ObjectDesignator.id == id
    }


class BelieveObject(ObjectDesignator):

    __tablename__ = "BelieveObject"

    id: Mapped[int] = mapped_column(ForeignKey("Object.id"), primary_key=True, init=False)

    __mapper_args__ = {
        "polymorphic_identity": __tablename__,
    }

    # def __init__(self, type: str, name: str):
    #     super().__init__(type, name)
