"""Implementation of base classes for orm modelling."""
import datetime
import getpass
import os
from dataclasses import field
from typing import Optional

import git
import rospkg
import sqlalchemy.sql.functions
from sqlalchemy import ForeignKey, String
from sqlalchemy.orm import DeclarativeBase, Mapped, MappedAsDataclass, mapped_column, Session, relationship, \
    declared_attr

from ..datastructures.enums import ObjectType


def get_pycram_version_from_git() -> Optional[str]:
    """
    Get the PyCRAM commit hash that is used to run this version.

    This assumes that you have gitpython installed and that the PyCRAM git repository on your system can be found
    with "roscd pycram".
    """

    r = rospkg.RosPack()
    repo = git.Repo(path=r.get_path('pycram'))
    return repo.head.object.hexsha


class _Base(DeclarativeBase, MappedAsDataclass):
    """Dummy class"""
    type_annotation_map = {
        str: String(255)
    }

    id: Mapped[int] = mapped_column(autoincrement=True, primary_key=True, init=False, nullable=False)
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
    def process_metadata_id(self) -> Mapped[int]:
        return mapped_column(ForeignKey(f'{ProcessMetaData.__tablename__}.id'), default=None, init=False)
    """Related MetaData Object to store information about the context of this experiment."""

    @declared_attr
    def process_metadata(self):
        return relationship(ProcessMetaData.__tablename__)
    """model relationship between foreign key in ProcessMetaData table and the ids of all inheriting
    tables"""


class MapperArgsMixin(MappedAsDataclass):
    """
    MapperArgsMixin stores __mapper_args__ information for certain subclass-tables.
    For information about Mixins, see https://docs.sqlalchemy.org/en/20/orm/declarative_mixins.html
    """

    __abstract__ = True

    @declared_attr
    def __mapper_args__(self):
        return {"polymorphic_identity": self.__tablename__}


class PositionMixin(MappedAsDataclass):
    """
    PositionMixin holds a foreign key column and its relationship to the referenced table.
    For information about Mixins, see https://docs.sqlalchemy.org/en/20/orm/declarative_mixins.html
    """

    __abstract__ = True
    position_to_init: bool = field(default=False, init=False)

    @declared_attr
    def position_id(self) -> Mapped[int]:
        return mapped_column(ForeignKey(f'{Position.__tablename__}.id'), init=self.position_to_init)

    @declared_attr
    def position(self):
        return relationship(Position.__tablename__, init=False)


class QuaternionMixin(MappedAsDataclass):
    """
    QuaternionMixin holds a foreign key column and its relationship to the referenced table.
    For information about Mixins, see https://docs.sqlalchemy.org/en/20/orm/declarative_mixins.html
    """

    __abstract__ = True
    orientation_to_init: bool = field(default=False, init=False)

    @declared_attr
    def orientation_id(self) -> Mapped[int]:
        return mapped_column(ForeignKey(f'{Quaternion.__tablename__}.id'), init=self.orientation_to_init)

    @declared_attr
    def orientation(self):
        return relationship(Quaternion.__tablename__, init=False)


class PoseMixin(MappedAsDataclass):
    """
    PoseMixin holds a foreign key column and its relationship to the referenced table.
    For information about Mixins, see https://docs.sqlalchemy.org/en/20/orm/declarative_mixins.html
    """

    __abstract__ = True
    pose_to_init: bool = field(default=False, init=False)

    @declared_attr
    def pose_id(self) -> Mapped[int]:
        return mapped_column(ForeignKey(f'{Pose.__tablename__}.id'), init=self.pose_to_init)

    @declared_attr
    def pose(self):
        return relationship(Pose.__tablename__, init=False)


class ProcessMetaData(_Base):
    """
    ProcessMetaData stores information about the context of this experiment.

    This class is a singleton and only one MetaData can exist per session.
    """

    created_at: Mapped[datetime.datetime] = mapped_column(server_default=sqlalchemy.sql.functions.current_timestamp(),
                                                          init=False)
    """The timestamp where this row got created. This is an aid for versioning."""

    created_by: Mapped[str] = mapped_column(default=getpass.getuser(), init=False)
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
        return self

    @classmethod
    def reset(cls):
        """Reset the singleton instance to None, s. t. next time the class is called a new instance is created."""
        cls._self = None


class Designator(Base):
    """ORM Class holding every performed action and motion serving as every performables and motions root."""

    @declared_attr
    def dtype(self) -> Mapped[str]:
        return mapped_column(String(255), nullable=False, init=False)

    @declared_attr
    def __mapper_args__(self):
        return {
            "polymorphic_on": "dtype",
        }


class Position(Base):
    """ORM Class for 3D positions."""

    x: Mapped[float]
    y: Mapped[float]
    z: Mapped[float]


class Quaternion(Base):
    """ORM Class for Quaternions."""

    x: Mapped[float]
    y: Mapped[float]
    z: Mapped[float]
    w: Mapped[float]


class Pose(PositionMixin, QuaternionMixin, Base):
    """ORM Class for Poses."""

    time: Mapped[datetime.datetime]
    frame: Mapped[str]


class Color(Base):
    """ORM Class for Colors."""

    r: Mapped[float]
    g: Mapped[float]
    b: Mapped[float]
    alpha: Mapped[float]


class RobotState(PoseMixin, Base):
    """ORM Representation of a robots state."""

    torso_height: Mapped[float]
    """The torso height of the robot."""

    type: Mapped[ObjectType]
    """The type of the robot."""
