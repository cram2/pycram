import tempfile
import unittest

import networkx as nx
from matplotlib import pyplot as plt
from networkx.drawing.nx_agraph import graphviz_layout
from owlready2 import get_ontology
from pycrap.parser import Parser


class ParserTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.onto = get_ontology("http://protege.stanford.edu/ontologies/pizza/pizza.owl").load()
        # cls.onto = get_ontology("http://www.ease-crc.org/ont/SOMA.owl").load()
        # cls.namespace = cls.onto.get_namespace("http://www.co-ode.org/ontologies/pizza/pizza.owl#")
        # cls.namespace.Pizza.comment = ["A pizza"]
        # cls.namespace.hasTopping.comment = ["Swaggy topping"]
        cls.file = tempfile.NamedTemporaryFile()
        cls.parser = Parser(cls.onto, "/home/tom_sch/.config/JetBrains/PyCharm2024.3/scratches/pizza.py")# cls.file.name)


    def test_create_graph(self):
        self.parser.create_dependency_graph()
        pos = graphviz_layout(self.parser.dependency_graph)
        nx.draw(self.parser.dependency_graph, with_labels=True, pos=pos)
        plt.show()
        print(nx.is_directed_acyclic_graph(self.parser.dependency_graph))
        print(*nx.simple_cycles(self.parser.dependency_graph))

    def test_parsing(self):
        self.parser.parse()

        with open(self.file.name, "r") as f:
            print(f.read())

        source = open(self.file.name).read() + "\n"
        compile(source, tempfile.NamedTemporaryFile().name, 'exec')



if __name__ == '__main__':
    unittest.main()
