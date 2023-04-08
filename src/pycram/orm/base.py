import sqlalchemy
import sqlalchemy.orm


class Base(sqlalchemy.orm.DeclarativeBase):
    """Base class to add orm functionality to all pycram mappings"""

    def __repr__(self):
        return f"{self.__module__}.{self.__class__.__name__}(" + \
               ", ".join([str(self.__getattribute__(c_attr.key)) for c_attr in
                          sqlalchemy.inspect(self).mapper.column_attrs]) + ")"
