import unittest
import pycrap
import inspect

from pycrap import Ontology


def recursive_subclasses(cls):
    """
    :param cls: The class.
    :return: A list of the classes subclasses.
    """
    return cls.__subclasses__() + [g for s in cls.__subclasses__() for g in recursive_subclasses(s)]


class CrapTestCase(unittest.TestCase):

    def setUp(self):
        self.ontology = Ontology()

    def test_creation(self):
        for cls in recursive_subclasses(pycrap.Base):
            cls: pycrap.Base
            cls.set_comment_to_docstring()
        self.assertTrue(len(pycrap.PhysicalObject.comment) > 0)

    def test_multiple_worlds(self):
        second_ontology = Ontology()
        cup1 = pycrap.Cup(namespace=self.ontology.ontology)
        cup2 = pycrap.Cup(namespace=second_ontology.ontology)
        self.assertEqual(len(list(self.ontology.individuals())), 1)
        self.assertEqual(len(list(second_ontology.individuals())), 1)
        self.assertNotEqual(cup1, cup2)


if __name__ == '__main__':
    unittest.main()