import unittest
import test_bullet_world
from pycram.designators.object_designator import *


class TestObjectDesignator(test_bullet_world.BulletWorldTest):

    def test_object_grounding(self):
        description = ObjectDesignatorDescription(["milk"], ["milk"])
        obj = description.ground()

        self.assertEqual(obj.name, "milk")
        self.assertEqual(obj.type, "milk")


if __name__ == '__main__':
    unittest.main()
