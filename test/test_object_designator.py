import unittest
from bullet_world_testcase import BulletWorldTestCase
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType


class TestObjectDesignator(BulletWorldTestCase):

    def test_object_grounding(self):
        description = ObjectDesignatorDescription(["milk"], [ObjectType.MILK])
        obj = description.ground()

        self.assertEqual(obj.name, "milk")
        self.assertEqual(obj.obj_type, ObjectType.MILK)

    def test_frozen_copy(self):
        description = ObjectDesignatorDescription(["milk"], [ObjectType.MILK])
        obj = description.ground()

        frozen_copy = obj.frozen_copy()
        self.assertEqual(obj.pose, frozen_copy.pose)


if __name__ == '__main__':
    unittest.main()
