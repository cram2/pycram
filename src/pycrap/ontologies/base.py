from owlready2 import Thing, ThingClass, ObjectProperty, FunctionalProperty, get_ontology, And, Or, Not, OneOf, Inverse, normstr, DatatypeProperty, TransitiveProperty, SymmetricProperty, AsymmetricProperty, ReflexiveProperty, IrreflexiveProperty, datetime
import tempfile


ontology_file = tempfile.NamedTemporaryFile()
ontology = get_ontology("file://" + ontology_file.name).load()
CRAX_ONTOLOGY_NAME = "PyCRAP"


class Base(Thing, metaclass=ThingClass):
    namespace = ontology


class BaseProperty(ObjectProperty):
    namespace = ontology


class BaseDatatype(DatatypeProperty):
    namespace = ontology
