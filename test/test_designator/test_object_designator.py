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


if __name__ == '__main__':
    unittest.main()
