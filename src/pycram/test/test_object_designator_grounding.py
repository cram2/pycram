from copy import deepcopy
from unittest import TestCase

from pycram.knowrob import knowrob
from pycram.object_designator import LocatedObjectDesignatorDescription, ObjectDesignator
from pycram.resolver import object_designator_grounding     # do not remove


class TestObjectDesignatorGrounding(TestCase):
    def setUp(self) -> None:
        knowrob.clear_beliefstate()
        knowrob.once("""
            kb_project([
                is_individual(a1), instance_of(a1, a),
                is_individual(a2), instance_of(a2, a)]),
            tf_logger_enable,
            mem_tf_set(a1, world, [1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 1.0], 4.0),
            mem_tf_set(a2, world, [2.0, 2.0, 2.0], [0.0, 0.0, 0.0, 1.0], 4.0)""")

    def tearDown(self) -> None:
        knowrob.clear_beliefstate()

    def test_ground_located_object_by_type(self):
        desc = LocatedObjectDesignatorDescription(type_="a")
        desig = ObjectDesignator(desc)
        all_solutions = self._get_all_solutions(desig)
        names = [sol["name"] for sol in all_solutions]
        self.assertSetEqual(set(names), {"a1", "a2"})

    @staticmethod
    def _get_all_solutions(desig):
        all_solutions = []
        while True:
            res = desig.reference()
            all_solutions.append(deepcopy(res))
            desig = desig.next_solution()
            if desig is None:
                break
        return all_solutions
