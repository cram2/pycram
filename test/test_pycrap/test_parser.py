import tempfile
import unittest

from owlready2 import get_ontology
from pycrap.parser import Parser


class ParserTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.onto = get_ontology("http://protege.stanford.edu/ontologies/pizza/pizza.owl").load()
        # cls.onto = get_ontology("http://www.ease-crc.org/ont/SOMA.owl").load()
        cls.namespace = cls.onto.get_namespace("http://www.co-ode.org/ontologies/pizza/pizza.owl#")
        # cls.namespace.Pizza.comment = ["A pizza"]
        # cls.namespace.hasTopping.comment = ["Swaggy topping"]
        cls.file = tempfile.NamedTemporaryFile()
        cls.parser = Parser(cls.onto, cls.file.name)


    def test_parse_restriction(self):
        concept = self.namespace.Pizza


    def test_parsing(self):
        self.parser.parse()

        with open(self.file.name, "r") as f:
            print(f.read())

        source = open(self.file.name).read() + "\n"
        compile(source, tempfile.NamedTemporaryFile().name, 'exec')



if __name__ == '__main__':
    unittest.main()
