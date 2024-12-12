import unittest

from pycrap.ontologies import DesignedFurniture, Surface
from pycrap.ontology_wrapper import OntologyWrapper


class TableConceptTestCase(unittest.TestCase):
    """
    Test some basic ontology queries.
    """

    def setUp(self):
        self.ontology = OntologyWrapper()

    def test_table_creation(self):
        table_without_parts = DesignedFurniture()
        table_with_parts = DesignedFurniture()
        table_top = Surface()
        table_with_parts.has_part = [table_top]
        result = list(self.ontology.search(is_a=DesignedFurniture, has_part=self.ontology.search(is_a=Surface)))
        self.assertEqual(len(result), 1)
        tables = list(self.ontology.search(type=DesignedFurniture))
        self.assertEqual(len(tables), 2)

    def tearDown(self):
        self.ontology.destroy_individuals()
