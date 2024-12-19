import unittest

from owlready2 import destroy_entity
from pycram.testing import EmptyBulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import (DesignedFurniture, Surface, PhysicalObject, ontology, Bowl,
                               Kitchen, PhysicalTask, Milk, Cereal, Refrigerator, Sink, Washer, Action)
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
        cereal = Object("cereal", Cereal, "breakfast_cereal.stl")
        milk = Object("milk", Milk, "milk.stl")


        used_classes = []
        for i in self.world.ontology.individuals():
            for i1 in i.is_a:
                used_classes.append(i1)

        for c in self.world.ontology.classes():
            if (c not in used_classes and c is not Refrigerator and c is not Kitchen
                    and c is not PhysicalTask and c is not PhysicalObject and c is not Milk and c is not Cereal
            and c is not Sink and c is not Washer):
                destroy_entity(c)

        refrigeratorTEST = Refrigerator(name="refrigeratorTEST", namespace=self.world.ontology.ontology)
        refrigeratorTEST1 = Refrigerator(name="refrigeratorTEST1", namespace=self.world.ontology.ontology)

        milk1TEST = Milk(name="milk1Test", namespace=self.world.ontology.ontology)
        kitchenTest = Kitchen(name="kitchenTest", namespace=self.world.ontology.ontology)
        physicalobjectTest = PhysicalObject(name="physicalobjectTest", namespace=self.world.ontology.ontology)
        physicalTask = PhysicalTask(name="physicalTaskTest", namespace=self.world.ontology.ontology)


        self.world.ontology.reason()

        for i in self.world.ontology.individuals():
            print(
                f"Individual: {i} is a: {i.is_a}, has_part: {i.has_part}, has predefined name: {i.has_predefined_name}"
                f" has predefined location: {i.has_predefined_location}, has origin location: {i.has_origin_location}",
                f" has location: {i.has_location}")














