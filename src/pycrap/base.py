from owlready2 import Thing
from .ontology import Ontology

default_pycrap_ontology = Ontology()

class Base(Thing):
    comment = __doc__
    namespace = default_pycrap_ontology.ontology

    @classmethod
    def set_comment_to_docstring(cls):
        cls.comment = cls.__doc__


class PhysicalObject(Base):
    """
    Any Object that has a proper space region. The prototypical physical object has also an associated mass, but the nature of its mass can greatly vary based on the epistemological status of the object (scientifically measured, subjectively possible, imaginary).
    """