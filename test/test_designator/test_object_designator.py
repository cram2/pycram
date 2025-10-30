import unittest

from krrood.entity_query_language.entity import an, entity, let, and_, contains
from krrood.entity_query_language.symbolic import symbolic_mode

from pycram.designator import EQLObjectDesignator, NamedObject
from pycram.designators.object_designator import *
from pycram.language import SequentialPlan
from pycram.testing import BulletWorldTestCase


class TestObjectDesignator(BulletWorldTestCase):

    def test_eql_designator(self):
        with symbolic_mode():
            milk_desig = EQLObjectDesignator(
                an(entity(obj := let(type_=Body, domain=self.world.bodies),
                            contains( obj.name.name, "milk")))
            )
        found_milks = list(milk_desig)
        self.assertEqual(1, len(found_milks))
        self.assertEqual("milk.stl", found_milks[0].name.name)
        self.assertEqual(self.world.get_body_by_name("milk.stl"), found_milks[0])

    def test_named_object(self):
        named_desig = NamedObject("milk.stl")
        plan = SequentialPlan(self.context,  named_desig)
        found_milks = list(named_desig)
        self.assertEqual(1, len(found_milks))
        self.assertEqual("milk.stl", found_milks[0].name.name)
        self.assertEqual(self.world.get_body_by_name("milk.stl"), found_milks[0])


if __name__ == '__main__':
    unittest.main()
