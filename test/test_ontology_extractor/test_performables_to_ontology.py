from pycram.ontology_extractor.performables_to_ontology import create_ontology_from_performables
from tempfile import NamedTemporaryFile
import owlready2
import unittest



class TestOntologyCreation(unittest.TestCase):
    def test_Performables_parsed(self):
        tmpfile = NamedTemporaryFile(mode="w+", suffix=".owl")
        create_ontology_from_performables(outputfile=tmpfile.name)
        ontology = owlready2.get_ontology(tmpfile.name).load()
        performable_class = next(filter(lambda c: c._name == 'Performable', ontology.classes()))
        self.assertTrue(len(performable_class.instances()) > 0)

    def test_Parameters_parsed(self):
        tmpfile = NamedTemporaryFile(mode="w+", suffix=".owl")
        create_ontology_from_performables(outputfile=tmpfile.name)
        ontology = owlready2.get_ontology(tmpfile.name).load()
        parameter_class = next(filter(lambda c: c._name == 'Parameter', ontology.classes()))
        self.assertTrue(len(parameter_class.instances()) > 0)

    def test_Enums_parsed(self):
        tmpfile = NamedTemporaryFile(mode="w+", suffix=".owl")
        create_ontology_from_performables(outputfile=tmpfile.name)
        ontology = owlready2.get_ontology(tmpfile.name).load()
        enum_class = next(filter(lambda c: c._name == 'Enum', ontology.ontology.classes()))
        self.assertTrue(len(enum_class.instances()) > 0)

if __name__ == '__main__':
    unittest.main()
