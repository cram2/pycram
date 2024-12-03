import tempfile
import unittest

import os

import networkx as nx
import matplotlib.pyplot as plt
from owlready2 import get_ontology
from pycrap.parser import OntologyParser, OntologiesParser


class OntologyParserTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # cls.onto = get_ontology("https://raw.githubusercontent.com/sorinar329/CuttingFood/refs/heads/main/owl/food_cutting.owl").load()
        # cls.onto = get_ontology("http://protege.stanford.edu/ontologies/pizza/pizza.owl").load()
        cls.onto = get_ontology("http://www.ease-crc.org/ont/SOMA.owl").load()
        #
        # pizza = cls.onto.get_namespace("http://www.co-ode.org/ontologies/pizza/pizza.owl#")
        # my_pizza = pizza.Pizza()
        # my_pizza.hasTopping = [pizza.CheeseTopping()]

        # cls.directory = tempfile.mkdtemp()
        cls.directory = "/home/tom_sch/playground/parsed_ontology"
        cls.parser = OntologyParser(cls.onto, cls.directory)


    def test_parsing(self):
        self.parser.parse()
        for file in os.listdir(self.directory):
            if file.endswith(".py"):
                with open(os.path.join(self.parser.path, file)) as f:
                    compile(f.read(), os.path.join(self.parser.path, file), 'exec')



class OntologiesParserTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ontologies = [get_ontology("https://raw.githubusercontent.com/sorinar329/CuttingFood/refs/heads/main/owl/food_cutting.owl").load(),
                          get_ontology("http://www.ease-crc.org/ont/SOMA.owl").load()]
        cls.directory = "/home/tom_sch/playground/parsed_ontology"
        cls.parser = OntologiesParser(cls.ontologies, cls.directory)


    def test_parsing(self):
        self.parser.create_ontologies()
        # nx.draw(self.parser.dependency_graph, with_labels=True)
        # plt.show()


if __name__ == '__main__':
    unittest.main()
