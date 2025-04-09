import unittest

from owlready2 import destroy_entity, get_ontology, Thing, FunctionalProperty, Imp, sync_reasoner_pellet
from pycram.testing import EmptyBulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import (DesignedFurniture, Surface, PhysicalObject, ontology, Bowl,
                               Kitchen, PhysicalTask, Milk, Cereal, Refrigerator, Sink, Washer, Action, is_part_of,
                               Drawer, Door, Agent, Location, Cabinet)
from pycrap.ontology_wrapper import OntologyWrapper
import os


class TableConceptTestCase(unittest.TestCase):
    """
    Test some basic ontology queries.
    """

    def setUp(self):
        self.ontology = OntologyWrapper()

    @unittest.skip(reason="Skipping test since it consumes to much memory")
    def test_rules(self):
        with self.ontology.ontology:

            kitchen = Kitchen()
            drawer = Drawer()
            milk = Milk()
            drawer.is_part_of = [kitchen]
            drawer.contains_object = [milk]
            rule = Imp()
            rule.set_as_rule("""is_part_of(?part, ?parent), contains_object(?part, ?object) -> contains_object(?parent, ?object)""")
            sync_reasoner_pellet(infer_property_values=True, infer_data_property_values=True)
            self.assertTrue(milk in kitchen.contains_object)

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

        print(result)
        #Drawer should be part of itself, cabinet and kitchen (reasoned)
        self.assertEqual(len(result), 3)

    def test_contains_object(self):
        cabinet = Cabinet("cabinet")
        drawer = Drawer("drawer")
        kitchen = Location("kitchen")

        kitchen.contains_object = [cabinet]
        cabinet.contains_object = [drawer]

        self.world.ontology.reason()
        result = kitchen.contains_object

        # Kitchen should contain cabinet and drawer (reasoned)
        self.assertEqual(len(result), 2)
        self.assertTrue(cabinet in result)
        self.assertTrue(drawer in result)
        self.assertTrue(kitchen not in result)

    def test_inverse_property(self):
        cabinet = Cabinet("cabinet")
        drawer = Drawer("drawer")
        kitchen = Location("kitchen")

        kitchen.contains_object = [cabinet]
        cabinet.contains_object = [drawer]

        self.world.ontology.reason()
        result = drawer.is_physically_contained_in

        # Drawer should be contained in cabinet and kitchen (reasoned)
        self.assertEqual(len(result), 2)
        self.assertTrue(cabinet in result)
        self.assertTrue(kitchen in result)
        self.assertTrue(drawer not in result)












