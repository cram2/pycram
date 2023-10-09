"""Implementation of base classes for orm modelling."""
import datetime
import logging
import os
from typing import Optional, List

import rospkg

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
    try:
        import git
    except ImportError:
        logging.warning("gitpython is not installed.")
        return None

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


class Base(_Base):
    """
    Base class to add orm functionality to all pycram mappings
    """
    __abstract__ = True

    metadata_id: Mapped[Optional[int]] = mapped_column(ForeignKey("MetaData.id"), default=None, init=False)
    """Related MetaData Object to store information about the context of this experiment."""

    @declared_attr
    def metadata_table_entry(self):
        return relationship("MetaData")
    """Declared attribute used to create a basic relationship pattern between the foreign keys of inheriting tables
    and the MetaData table itself in order to avoid complex queries and joins."""

    def __repr__(self):
        return f"{self.__module__}.{self.__class__.__name__}(" + ", ".join(
            [str(self.__getattribute__(c_attr.key)) for c_attr in inspect(self).mapper.column_attrs]) + ")"


class MetaData(MappedAsDataclass, _Base):
    """
    MetaData stores information about the context of this experiment.

    This class is a singleton and only one MetaData can exist per session.
    """

    __tablename__ = "MetaData"

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

    __tablename__ = "Position"

    x: Mapped[float]
    y: Mapped[float]
    z: Mapped[float]
    metadata_id: Mapped[Optional[int]] = mapped_column(ForeignKey("MetaData.id"), default=None)

    # def __init__(self, x: int, y: int, z: int, metadata_id: Optional[int] = None):
    #     super().__init__()
    #     self.x = x
    #     self.y = y
    #     self.z = z
    #     self.metadata_id = metadata_id


class Quaternion(MappedAsDataclass, Base):
    """ORM Class for Quaternions."""

    __tablename__ = "Quaternion"

    x: Mapped[float]
    y: Mapped[float]
    z: Mapped[float]
    w: Mapped[float]
    metadata_id: Mapped[Optional[int]] = mapped_column(ForeignKey("MetaData.id"), default=None)

    # def __init__(self, x: float, y: float, z: float, w: float, metadata_id: Optional[int] = None):
    #     super().__init__()
    #     self.x = x
    #     self.y = y
    #     self.z = z
    #     self.w = w
    #     self.metadata_id = metadata_id


class Color(MappedAsDataclass, Base):
    """ORM Class for Colors."""

    __tablename__ = "Color"

    r: Mapped[float]
    g: Mapped[float]
    b: Mapped[float]
    alpha: Mapped[float]

    # def __init__(self, r: float, g: float, b: float, alpha: float):
    #     super().__init__()
    #     self.r = r
    #     self.g = g
    #     self.b = b
    #     self.alpha = alpha


class RobotState(MappedAsDataclass, Base):
    """ORM Representation of a robots state."""

    __tablename__ = "RobotState"

    position: Mapped[int] = mapped_column(ForeignKey("Position.id"), init=False)
    position_table_entry: Mapped["Position"] = relationship(init=False)
    """The position of the robot."""

    orientation: Mapped[int] = mapped_column(ForeignKey("Quaternion.id"), init=False)
    orientation_table_entry: Mapped["Quaternion"] = relationship(init=False)
    """The orientation of the robot."""

    torso_height: Mapped[float] = mapped_column(init=False)
    """The torso height of the robot."""

    type: Mapped[str] = mapped_column(init=False)
    """The type of the robot."""
