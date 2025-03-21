import unittest

from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.partial_designator import PartialDesignator
from pycram.datastructures.pose import Pose
from pycram.testing import BulletWorldTestCase
from pycram.designators.action_designator import PickUpAction, PickUpAction, SetGripperAction, \
    MoveTorsoAction, NavigateAction, MoveTorsoActionDescription, NavigateActionDescription, PickUpActionDescription
from pycram.designators.object_designator import BelieveObject
from pycram.datastructures.enums import Arms, Grasp, GripperState, TorsoState
from pycrap.ontologies import Milk
from pycram.utils import is_iterable, lazy_product
from pycram.process_module import simulated_robot

class TestPartialDesignator(BulletWorldTestCase):
    def test_partial_desig_construction(self):
        test_object = BelieveObject(names=["milk"])
        partial_desig = PartialDesignator(PickUpAction, test_object, arm=Arms.RIGHT)
        self.assertEqual(partial_desig.performable, PickUpAction)
        self.assertEqual(partial_desig.kwargs, {"arm": Arms.RIGHT, "object_designator": test_object,
                                                "grasp_description": None, 'prepose_distance': None})

    def test_partial_desig_construction_none(self):
        partial_desig =  PartialDesignator(PickUpAction,None, arm=Arms.RIGHT)
        self.assertEqual(partial_desig.performable, PickUpAction)
        self.assertEqual(partial_desig.kwargs, {"arm": Arms.RIGHT, "object_designator": None,
                                                "grasp_description": None, 'prepose_distance': None})

    def test_partial_desig_call(self):
        partial_desig =  PartialDesignator(PickUpAction, None, arm=Arms.RIGHT)
        grasp_description = GraspDescription(Grasp.FRONT, None, False)
        new_partial_desig = partial_desig(grasp_description=grasp_description)
        self.assertEqual(new_partial_desig.performable, PickUpAction)
        self.assertEqual({"arm": Arms.RIGHT, "grasp_description": grasp_description, "object_designator": None,
                          'prepose_distance': None}, new_partial_desig.kwargs)

    def test_partial_desig_missing_params(self):
        partial_desig =  PartialDesignator(PickUpAction, None, arm=Arms.RIGHT)
        missing_params = partial_desig.missing_parameter()
        self.assertTrue("object_designator" in missing_params and "grasp_description" in missing_params)

        grasp_description = GraspDescription(Grasp.FRONT, None, False)
        new_partial = partial_desig(grasp_description=grasp_description)
        missing_params = new_partial.missing_parameter()
        self.assertEqual(['object_designator', "prepose_distance"], missing_params)

    def test_is_iterable(self):
        self.assertTrue(is_iterable([1, 2, 3]))
        self.assertFalse(is_iterable(1))

    def test_partial_desig_permutations(self):
        tp = PartialDesignator(SetGripperAction, [Arms.LEFT, Arms.RIGHT], motion=[GripperState.OPEN, GripperState.CLOSE])
        permutations = tp.generate_permutations()
        self.assertEqual([(Arms.LEFT, GripperState.OPEN), (Arms.LEFT, GripperState.CLOSE),
                          (Arms.RIGHT, GripperState.OPEN), (Arms.RIGHT, GripperState.CLOSE)], [tuple(p.values()) for p in permutations])

    def test_partial_desig_permutation_dict(self):
        tp = PartialDesignator(SetGripperAction, [Arms.LEFT, Arms.RIGHT], motion=[GripperState.OPEN, GripperState.CLOSE])
        permutations = tp.generate_permutations()
        self.assertEqual({"gripper": Arms.LEFT, "motion": GripperState.OPEN}, list(permutations)[0])

    def test_partial_desig_iter(self):
        test_object = BelieveObject(names=["milk"])
        test_object_resolved = test_object.resolve()
        partial_desig =  PartialDesignator(PickUpAction, test_object, arm=[Arms.RIGHT, Arms.LEFT])
        grasp_description_front = GraspDescription(Grasp.FRONT, None, False)
        grasp_description_top = GraspDescription(Grasp.FRONT, None, False)
        performables = list(partial_desig(grasp_description=[grasp_description_front,grasp_description_top]))
        self.assertEqual(4, len(performables))
        self.assertTrue(all([isinstance(p, PickUpAction) for p in performables]))
        self.assertEqual([p.arm for p in performables], [Arms.RIGHT, Arms.RIGHT, Arms.LEFT, Arms.LEFT])
        self.assertEqual([p.grasp_description for p in performables], [grasp_description_front, grasp_description_top, grasp_description_front, grasp_description_top])
        self.assertEqual([p.object_designator for p in performables], [test_object_resolved] * 4)

class TestPartialActions(BulletWorldTestCase):

    def test_partial_movetorso_action(self):
        move1 = MoveTorsoActionDescription(TorsoState.HIGH).resolve()
        self.assertEqual(move1.torso_state, TorsoState.HIGH)
        move2 = MoveTorsoActionDescription([TorsoState.HIGH, TorsoState.MID])
        for action in move2:
            self.assertTrue(action.torso_state in [TorsoState.HIGH, TorsoState.MID])

    def test_partial_navigate_action_perform(self):
        with simulated_robot:
            move1 = NavigateActionDescription(Pose([1, 0, 0])).resolve().perform()
            self.assertEqual(self.robot.pose.position_as_list(), [1, 0, 0])

    def test_partial_navigate_action_multiple(self):
        nav = NavigateActionDescription([Pose([1, 0, 0]), Pose([2, 0, 0]), Pose([3, 0, 0])])
        nav_goals = [[1, 0, 0], [2, 0, 0], [3, 0, 0]]
        for i, action in enumerate(nav):
            with simulated_robot:
                action.perform()
                self.assertEqual(self.robot.pose.position_as_list(), nav_goals[i])

    def test_partial_pickup_action(self):
        milk_desig = BelieveObject(names=["milk"])
        grasp_description = GraspDescription(Grasp.FRONT, None, False)
        pick = PickUpActionDescription(milk_desig, [Arms.LEFT, Arms.RIGHT], grasp_description)
        pick_action = pick.resolve()
        self.assertEqual(pick_action.object_designator.obj_type, Milk)
        self.assertEqual(pick_action.arm, Arms.LEFT)
        self.assertEqual(pick_action.grasp_description, grasp_description)

    def test_partial_pickup_action_insert_param(self):
        milk_desig = BelieveObject(names=["milk"])
        grasp_description = GraspDescription(Grasp.FRONT, None, False)
        pick = PickUpActionDescription(milk_desig, [Arms.LEFT, Arms.RIGHT])
        pick_action = pick(grasp_description=grasp_description).resolve()
        self.assertEqual(pick_action.grasp_description, grasp_description)


class TestLazyProduct(unittest.TestCase):

    def test_lazy_product_result(self):
        l1 = [0, 1]
        l2 = [3, 4]
        self.assertEqual(list(lazy_product(l1, l2)), [(0,3), (0,4), (1,3), (1,4)])

    def test_lazy_product_single_input(self):
        l1 = [0, 1]
        self.assertEqual(list(lazy_product(l1)), [(0,), (1,)])

    def test_lazy_product_lazy_evaluate(self):
        def bad_generator():
            for i in range(10):
                if i == 5:
                    raise RuntimeError()
                yield i
        l1 = iter(bad_generator())
        l2 = iter(bad_generator())
        res = next(lazy_product(l1, l2))
        self.assertEqual(res, (0,0))

    def test_lazy_product_error(self):
        def bad_generator():
            for i in range(10):
                if i == 5:
                    raise RuntimeError()
                yield i
        self.assertRaises(RuntimeError, lambda: list(lazy_product(bad_generator())))
