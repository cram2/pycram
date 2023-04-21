"""Implementation of base classes for orm modelling."""

import sqlalchemy
import sqlalchemy.orm


class Base(sqlalchemy.orm.DeclarativeBase):
    """Base class to add orm functionality to all pycram mappings"""

    def __repr__(self):
        return f"{self.__module__}.{self.__class__.__name__}(" + \
               ", ".join([str(self.__getattribute__(c_attr.key)) for c_attr in
                          sqlalchemy.inspect(self).mapper.column_attrs]) + ")"


class Position(Base):
    """ORM Class for 3D positions."""

    __tablename__ = "Position"

    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    x = sqlalchemy.Column(sqlalchemy.types.Float)
    y = sqlalchemy.Column(sqlalchemy.types.Float)
    z = sqlalchemy.Column(sqlalchemy.types.Float)

    def __init__(self, x: int, y: int, z: int):
        super().__init__()
        self.x = x
        self.y = y
        self.z = z


class Quaternion(Base):
    """ORM Class for Quaternions."""

    __tablename__ = "Quaternion"

    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    x = sqlalchemy.Column(sqlalchemy.types.Float)
    y = sqlalchemy.Column(sqlalchemy.types.Float)
    z = sqlalchemy.Column(sqlalchemy.types.Float)
    w = sqlalchemy.Column(sqlalchemy.types.Float)

    def __init__(self, x: float, y: float, z: float, w: float):
        super().__init__()
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class Color(Base):
    """ORM Class for Colors."""

    __tablename__ = "Color"

    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    r = sqlalchemy.Column(sqlalchemy.types.Float)
    g = sqlalchemy.Column(sqlalchemy.types.Float)
    b = sqlalchemy.Column(sqlalchemy.types.Float)
    alpha = sqlalchemy.Column(sqlalchemy.types.Float)

    def __init__(self, r: float, g: float, b: float, alpha: float):
        super().__init__()
        self.r = r
        self.g = g
        self.b = b
        self.alpha = alpha

