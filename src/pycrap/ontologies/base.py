from owlready2 import *
import tempfile


ontology_file = tempfile.NamedTemporaryFile()
ontology = owlready2.get_ontology("file://" + ontology_file.name).load()


class Base(Thing):
    namespace = ontology


class BaseProperty(ObjectProperty):
    namespace = ontology


