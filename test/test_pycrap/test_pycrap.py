import unittest
from unittest import skip

import pycrap
import inspect

from pycrap.ontologies import Base, Cup
from pycrap.ontology_wrapper import OntologyWrapper


def recursive_subclasses(cls):
    """
    :param cls: The class.
    :return: A list of the classes subclasses.
    """
    return cls.__subclasses__() + [g for s in cls.__subclasses__() for g in recursive_subclasses(s)]


class CrapTestCase(unittest.TestCase):

    def setUp(self):
        self.ontology = OntologyWrapper()

    @skip("Not working anymore as now we use one ontology")
    def test_multiple_worlds(self):
        second_ontology = OntologyWrapper()
        cup1 = Cup(namespace=self.ontology.ontology)
        cup2 = Cup(namespace=second_ontology.ontology)
        self.assertEqual(len(list(self.ontology.individuals())), 1)
        self.assertEqual(len(list(second_ontology.individuals())), 1)
        self.assertNotEqual(cup1, cup2)


if __name__ == '__main__':
    unittest.main()