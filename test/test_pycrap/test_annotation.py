import unittest

from owlready2 import destroy_entity
from pycram.testing import EmptyBulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import (DesignedFurniture, Surface, PhysicalObject, ontology, Bowl,
                               Kitchen, PhysicalTask, Milk, Cereal, Refrigerator, Sink, Washer, Action, is_part_of,
                               Drawer, Door, Agent, Location, Cabinet)
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


class AnnotationTestCase(EmptyBulletWorldTestCase):

    def test_reasoner(self):
        cabinet = Cabinet("cabinet")
        drawer = Drawer("drawer")
        kitchen = Location("kitchen")

        cabinet.is_part_of = [kitchen]
        drawer.is_part_of = [cabinet]

        self.world.ontology.reason()
        result = drawer.is_part_of

        #Drawer should be part of itself, cabinet and kitchen (reasoned)
        self.assertEqual(len(result), 3)












