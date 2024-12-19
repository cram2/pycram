from owlready2 import Thing, ThingClass, ObjectProperty, get_ontology, And, Or, Not, OneOf, Inverse, normstr, \
    DatatypeProperty, TransitiveProperty, SymmetricProperty, AsymmetricProperty, ReflexiveProperty, IrreflexiveProperty, \
    datetime, Imp
import tempfile


ontology_file = tempfile.NamedTemporaryFile()
ontology = get_ontology("file://" + ontology_file.name).load()


class Base(Thing, metaclass=ThingClass):
    namespace = ontology


class BaseProperty(ObjectProperty):
    namespace = ontology

class BaseDatatype(DatatypeProperty):
    namespace = ontology


