import unittest

from pycram.testing import EmptyBulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycrap import Table, Ontology, Board, PhysicalObject, has_part


class TableConceptTestCase(unittest.TestCase):

    def setUp(self):
        self.ontology = Ontology()

    def test_table_creation(self):
        table_without_parts = Table()
        table_with_parts = Table()
        table_top = Board()
        table_with_parts.has_part = [table_top]
        result = list(self.ontology.search(is_a=Table, has_part=self.ontology.search(is_a=Board)))
        self.assertEqual(len(result), 1)




class AnnotationTestCase(EmptyBulletWorldTestCase):

    def test_tagging_of_tables(self):
        table = Object("table", Table, "table" + self.extension)
        print(type(table.description))
