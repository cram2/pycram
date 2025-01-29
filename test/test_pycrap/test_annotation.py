import unittest

from owlready2 import destroy_entity
from pycram.testing import EmptyBulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import (DesignedFurniture, Surface, PhysicalObject, ontology, Bowl,
                               Kitchen, PhysicalTask, Milk, Cereal, Refrigerator, Sink, Washer, Action, is_part_of,
                               Drawer, Door, Agent)
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

    def test_parse_urdf_description(self):
        table = Object("kitchen", PhysicalObject, "kitchen-small" + self.extension)
        robot = Object("pr2", Agent, f"pr2{self.extension}")
        cereal = Object("cereal", Cereal, "breakfast_cereal.stl")
        milk = Object("milk", Milk, "milk.stl")
        apt = Object("apartment", Milk, "apartment" + self.extension)


        used_classes = []
        for i in self.world.ontology.individuals():
            for i1 in i.is_a:
                used_classes.append(i1)

        for c in self.world.ontology.classes():
            if (c not in used_classes and c is not Refrigerator and c is not Kitchen
                    and c is not PhysicalTask and c is not PhysicalObject and c is not Milk and c is not Cereal
            and c is not Sink and c is not Washer):
                destroy_entity(c)

        #self.world.ontology.reason()


        sink_area = self.world.ontology.search(iri="*sink_area")[0]

        for i in self.world.ontology.search(type=Drawer, is_part_of=(self.world.ontology.search(iri="*sink_area")[0])):
            for d in self.world.ontology.search(type=Door, is_part_of=i):
                print(d)











