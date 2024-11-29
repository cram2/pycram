import tempfile
import unittest

import os
from owlready2 import get_ontology
from pycrap.parser import Parser


class ParserTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # cls.onto = get_ontology("https://raw.githubusercontent.com/sorinar329/CuttingFood/refs/heads/main/owl/food_cutting.owl").load()
        cls.onto = get_ontology("http://protege.stanford.edu/ontologies/pizza/pizza.owl").load()
        # cls.onto = get_ontology("http://www.ease-crc.org/ont/SOMA.owl").load()
        #
        # pizza = cls.onto.get_namespace("http://www.co-ode.org/ontologies/pizza/pizza.owl#")
        # my_pizza = pizza.Pizza()
        # my_pizza.hasTopping = [pizza.CheeseTopping()]

        cls.directory = tempfile.mkdtemp()
        cls.parser = Parser(cls.onto, cls.directory)


    def test_parsing(self):
        self.parser.parse()
        for file in os.listdir(self.directory):
            if file.endswith(".py"):
                with open(os.path.join(self.parser.path, file)) as f:
                    compile(f.read(), os.path.join(self.parser.path, file), 'exec')




if __name__ == '__main__':
    unittest.main()
