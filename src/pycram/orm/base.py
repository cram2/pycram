"""Implementation of base classes for orm modelling."""
import datetime
import logging
import os
from typing import Optional, List

import rospkg
import git

from sqlalchemy import ForeignKey, String, inspect
from sqlalchemy.orm import DeclarativeBase, Mapped, MappedAsDataclass, mapped_column, Session, relationship, \
    declared_attr
import sqlalchemy.sql.functions
import sqlalchemy.engine


def get_pycram_version_from_git() -> Optional[str]:
    """
    Get the PyCRAM commit hash that is used to run this version.

    This assumes that you have gitpython installed and that the PyCRAM git repository on your system can be found
    with "roscd pycram".
    """

    r = rospkg.RosPack()
    repo = git.Repo(path=r.get_path('pycram'))
    return repo.head.object.hexsha


class _Base(DeclarativeBase):
    """Dummy class"""
    type_annotation_map = {
        str: String(255)
    }

    id: Mapped[int] = mapped_column(autoincrement=True, primary_key=True, init=False)
    """Unique integer ID as auto incremented primary key."""

    @declared_attr
    def __tablename__(self):
        return self.__name__


class Base(_Base):
    """
    Base class to add orm functionality to all pycram mappings
    """
    __abstract__ = True

    @declared_attr
    def processed_metadata_id(self) -> Mapped[Optional[int]]:
        return mapped_column(ForeignKey("ProcessedMetaData.id"), default=None, init=False)
    """Related MetaData Object to store information about the context of this experiment."""

    @declared_attr
    def processed_metadata(self):
        return relationship("ProcessedMetaData")
    """model relationship between foreign key in ProcessedMetaData table and the ids of all inheriting
    tables"""

    def __repr__(self):
        return f"{self.__module__}.{self.__class__.__name__}(" + ", ".join(
            [str(self.__getattribute__(c_attr.key)) for c_attr in inspect(self).mapper.column_attrs]) + ")"


class MapperArgsMixin:
    """
    MapperArgsMixin stores __mapper_args__ information for certain subclass-tables.
    For information about Mixins, see https://docs.sqlalchemy.org/en/13/orm/extensions/declarative/mixins.html
    """

    __abstract__ = True

    @declared_attr
    def __mapper_args__(self):
        return {"polymorphic_identity": self.__tablename__}


class ObjectMixin:
    """
    ObjectMixin holds a foreign key column and its relationship to the referenced table.
    For information about Mixins, see https://docs.sqlalchemy.org/en/13/orm/extensions/declarative/mixins.html
    """

    __abstract__ = True

    @declared_attr
    def object(self) -> Mapped[int]:
        return mapped_column(ForeignKey("Object.id"), init=False)

    @declared_attr
    def object_table_entry(self):
        return relationship("Object", init=False)


class PositionMixin:
    """
    PositionMixin holds a foreign key column and its relationship to the referenced table.
    For information about Mixins, see https://docs.sqlalchemy.org/en/13/orm/extensions/declarative/mixins.html
    """

    __abstract__ = True

    @declared_attr
    def position(self) -> Mapped[int]:
        return mapped_column(ForeignKey("Position.id"), init=False)

    @declared_attr
    def position_table_entry(self):
        return relationship("Position", init=False)


class QuaternionMixin:
    """
    QuaternionMixin holds a foreign key column and its relationship to the referenced table.
    For information about Mixins, see https://docs.sqlalchemy.org/en/13/orm/extensions/declarative/mixins.html
    """

    __abstract__ = True

    @declared_attr
    def orientation(self) -> Mapped[int]:
        return mapped_column(ForeignKey("Quaternion.id"), init=False)

    @declared_attr
    def orientation_table_entry(self):
        return relationship("Quaternion", init=False)


class ProcessedMetaData(MappedAsDataclass, _Base):
    """
    ProcessedMetaData stores information about the context of this experiment.

    This class is a singleton and only one MetaData can exist per session.
    """

    created_at: Mapped[datetime.datetime] = mapped_column(server_default=sqlalchemy.sql.functions.current_timestamp(),
                                                          init=False)
    """The timestamp where this row got created. This is an aid for versioning."""

    created_by: Mapped[str] = mapped_column(default=os.getlogin(), init=False)
    """The user that created the experiment."""

    description: Mapped[str] = mapped_column(init=False)
    """A description of the purpose (?) of this experiment."""

    pycram_version: Mapped[str] = mapped_column(default=get_pycram_version_from_git(),
                                                nullable=True, init=False)
    """The PyCRAM version used to generate this row."""

    _self = None
    """The singleton instance."""

    def __new__(cls):
        if cls._self is None:
            cls._self = super().__new__(cls)
        return cls._self

    def committed(self):
        """Return if this object is in the database or not."""
        return self.id is not None

    def insert(self, session: Session):
        """Insert this into the database using the session. Skipped if it already is inserted."""
        if not self.committed():
            session.add(self)
            session.commit()
        return self

    @classmethod
    def reset(cls):
        """Reset the singleton instance to None, s. t. next time the class is called a new instance is created."""
        cls._self = None


class Position(MappedAsDataclass, Base):
    """ORM Class for 3D positions."""

    x: Mapped[float]
    y: Mapped[float]
    z: Mapped[float]


class Quaternion(MappedAsDataclass, Base):
    """ORM Class for Quaternions."""

    x: Mapped[float]
    y: Mapped[float]
    z: Mapped[float]
    w: Mapped[float]


class Color(MappedAsDataclass, Base):
    """ORM Class for Colors."""

    r: Mapped[float]
    g: Mapped[float]
    b: Mapped[float]
    alpha: Mapped[float]


#TODO remove init=False in RobotState, change Designators accordingly
class RobotState(MappedAsDataclass, Base):
    """ORM Representation of a robots state."""

    position_id: Mapped[int] = mapped_column(ForeignKey("Position.id"), init=False)
    position: Mapped["Position"] = relationship(init=False)
    """The position of the robot."""

    orientation_id: Mapped[int] = mapped_column(ForeignKey("Quaternion.id"), init=False)
    orientation: Mapped["Quaternion"] = relationship(init=False)
    """The orientation of the robot."""

    torso_height: Mapped[float] = mapped_column(init=False)
    """The torso height of the robot."""

    type: Mapped[str] = mapped_column(init=False)
    """The type of the robot."""
