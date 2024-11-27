import unittest
from pycram.testing import BulletWorldTestCase, EmptyBulletWorldTestCase
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType
from pycrap import Milk, Food, Cereal


class TestObjectDesignator(BulletWorldTestCase):

    def test_object_grounding(self):
        description = ObjectDesignatorDescription(["milk"], [Milk])
        obj = description.ground()

        self.assertEqual(obj.name, "milk")
        self.assertEqual(obj.obj_type, Milk)

    def test_frozen_copy(self):
        description = ObjectDesignatorDescription(["milk"], [Milk])
        obj = description.ground()

        frozen_copy = obj.frozen_copy()
        self.assertEqual(obj.pose, frozen_copy.pose)


class OntologyObjectDesignatorDescriptionTestCase(BulletWorldTestCase):

    def test_type_query_for_food(self):
        self.world.ontology.reason()
        odd = OntologyObjectDesignatorDescription(self.world.ontology.search(type=Food))
        self.assertEqual(len(odd.search_result), 2)
        result_in_world = list(odd.__iter__())
        self.assertEqual(len(result_in_world), 2)




if __name__ == '__main__':
    unittest.main()
