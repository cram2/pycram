import sqlalchemy.orm


class Base(sqlalchemy.orm.DeclarativeBase):
    """Base class to add orm functionality for all pycram mappings"""
