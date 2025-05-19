from sqlalchemy import  types


class StringType(types.TypeDecorator):
    """
    This type represents a physical object type.
    The database representation of this is a string while the in memory type is the instance of PhysicalObject.
    """
    cache_ok = True
    impl = types.String

    def process_bind_param(self, value, dialect):
        return value.__class__.__name__

    def copy(self, **kw):
        return self.__class__(**kw)