from dataclasses import dataclass, field, Field
from random_events.variable import Integer, Continuous, Symbolic, SetElement


def metadata_of_field(field: Field):
    """
    Get the pycram metadata of a field. If the field has no pycram specific metadata,
    a new FieldMetaData object is created.
    :param field: The field to get the metadata from.
    :return: The pycram metadata of the field.
    """
    return field.metadata.get("pycram", PycramMetaData())

@dataclass
class PycramMetaData:
    """
    Class to store metadata for fields in pycram datastructures.
    This class contains pycram specific functionality for fields.
    """

    is_parameter: bool = True
    """
    Whether this field is a parameter (adjustable value) or not.
    """

def variable_of_field(f: Field, dtype: type):
    """
    Get the random events variable of a field.
    """
    if issubclass(dtype, int):
        return Integer(f.name)
    elif issubclass(dtype, float):
        return Continuous(f.name)
    elif issubclass(dtype, SetElement):
        return Symbolic(f.name, dtype)
    else:
        raise ValueError(f"Unsupported type {dtype}")
