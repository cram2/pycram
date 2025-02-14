import unittest

from pycram.designators.object_designator import *
from pycram.testing import BulletWorldTestCase
from pycrap.ontologies import Milk, Food


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
        result = self.world.ontology.search(type=Food)
        self.assertEqual(len(result), 2)



if __name__ == '__main__':
    unittest.main()
