import tempfile
import unittest

import os

import networkx as nx
import matplotlib.pyplot as plt
from owlready2 import get_ontology
from pycrap.parser import OntologiesParser



class OntologiesParserTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ontologies = [get_ontology("http://www.ease-crc.org/ont/SOMA.owl").load()]
        cls.directory = tempfile.mkdtemp()
        # cls.directory = os.path.join(os.path.expanduser("~") ,"playground/ontologies")
        cls.parser = OntologiesParser(cls.ontologies, cls.directory)

    def test_parsing(self):
        self.parser.parse()
        for file in os.listdir(self.directory):
            if file.endswith(".py"):
                with open(os.path.join(self.parser.path, file)) as f:
                    compile(f.read(), os.path.join(self.parser.path, file), 'exec')



if __name__ == '__main__':
    unittest.main()
