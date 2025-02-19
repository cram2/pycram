from pycram.datastructures.partial_designator import PartialDesignator
from pycram.datastructures.pose import Pose
from pycram.testing import BulletWorldTestCase
from pycram.designators.action_designator import PickUpAction, PickUpActionPerformable, SetGripperActionPerformable, \
    MoveTorsoAction, NavigateAction
from pycram.designators.object_designator import BelieveObject
from pycram.datastructures.enums import Arms, Grasp, GripperState, TorsoState
from pycrap.ontologies import Milk
from pycram.process_module import simulated_robot

class TestPartialDesignator(BulletWorldTestCase):
    def test_partial_desig_construction(self):
        test_object = BelieveObject(names=["milk"])
        partial_desig = PartialDesignator(PickUpActionPerformable, test_object, arm=Arms.RIGHT)
        self.assertEqual(partial_desig.performable, PickUpActionPerformable)
        self.assertEqual(partial_desig.kwargs, {"arm": Arms.RIGHT, "object_designator": test_object,
                                                "grasp": None, 'prepose_distance': None})

    def test_partial_desig_construction_none(self):
        partial_desig = PartialDesignator(PickUpActionPerformable, None, arm=Arms.RIGHT)
        self.assertEqual(partial_desig.performable, PickUpActionPerformable)
        self.assertEqual(partial_desig.kwargs, {"arm": Arms.RIGHT, "object_designator": None,
                                                "grasp": None, 'prepose_distance': None})

    def test_partial_desig_call(self):
        partial_desig = PartialDesignator(PickUpActionPerformable, None, arm=Arms.RIGHT)
        new_partial_desig = partial_desig(grasp=Grasp.FRONT)
        self.assertEqual(new_partial_desig.performable, PickUpActionPerformable)
        self.assertEqual({"arm": Arms.RIGHT, "grasp": Grasp.FRONT, "object_designator": None,
                          'prepose_distance': None}, new_partial_desig.kwargs)

    def test_partial_desig_missing_params(self):
        partial_desig = PartialDesignator(PickUpActionPerformable, None, arm=Arms.RIGHT)
        missing_params = partial_desig.missing_parameter()
        self.assertTrue("object_designator" in missing_params and "grasp" in missing_params)

        new_partial = partial_desig(grasp=Grasp.FRONT)
        missing_params = new_partial.missing_parameter()
        self.assertEqual(['object_designator', 'prepose_distance'], missing_params)

    def test_is_iterable(self):
        self.assertTrue(PartialDesignator._is_iterable([1, 2, 3]))
        self.assertFalse(PartialDesignator._is_iterable(1))

    def test_partial_desig_permutations(self):
        tp = PartialDesignator(SetGripperActionPerformable, [Arms.LEFT, Arms.RIGHT],
                               motion=[GripperState.OPEN, GripperState.CLOSE])
        permutations = tp.generate_permutations()
        self.assertEqual([(Arms.LEFT, GripperState.OPEN), (Arms.LEFT, GripperState.CLOSE),
                          (Arms.RIGHT, GripperState.OPEN), (Arms.RIGHT, GripperState.CLOSE)], list(permutations))

    def test_partial_desig_iter(self):
        test_object = BelieveObject(names=["milk"])
        test_object_resolved = test_object.resolve()
        partial_desig = PartialDesignator(PickUpActionPerformable, test_object, arm=[Arms.RIGHT, Arms.LEFT])
        performables = list(partial_desig(grasp=[Grasp.FRONT, Grasp.TOP]))
        self.assertEqual(4, len(performables))
        self.assertTrue(all([isinstance(p, PickUpActionPerformable) for p in performables]))
        self.assertEqual([p.arm for p in performables], [Arms.RIGHT, Arms.RIGHT, Arms.LEFT, Arms.LEFT])
        self.assertEqual([p.grasp for p in performables], [Grasp.FRONT, Grasp.TOP, Grasp.FRONT, Grasp.TOP])
        self.assertEqual([p.object_designator for p in performables], [test_object_resolved] * 4)

class TestPartialActions(BulletWorldTestCase):

    def test_partial_movetorso_action(self):
        move1 = MoveTorsoAction(TorsoState.HIGH).resolve()
        self.assertEqual(move1.torso_state, TorsoState.HIGH)
        move2 = MoveTorsoAction([TorsoState.HIGH, TorsoState.MID])
        for action in move2:
            self.assertTrue(action.torso_state in [TorsoState.HIGH, TorsoState.MID])

    def test_partial_navigate_action_perform(self):
        with simulated_robot:
            move1 = NavigateAction(Pose([1, 0, 0])).resolve().perform()
            self.assertEqual(self.robot.pose.position_as_list(), [1, 0, 0])

    def test_partial_navigate_action_multiple(self):
        nav = NavigateAction([Pose([1, 0, 0]), Pose([2, 0, 0]), Pose([3, 0, 0])])
        nav_goals = [[1, 0, 0], [2, 0, 0], [3, 0, 0]]
        for i, action in enumerate(nav):
            with simulated_robot:
                action.perform()
                self.assertEqual(self.robot.pose.position_as_list(), nav_goals[i])

    def test_partial_pickup_action(self):
        milk_desig = BelieveObject(names=["milk"])
        pick = PickUpAction(milk_desig, [Arms.LEFT, Arms.RIGHT], Grasp.FRONT)
        pick_action = pick.resolve()
        self.assertEqual(pick_action.object_designator.obj_type, Milk)
        self.assertEqual(pick_action.arm, Arms.LEFT)
        self.assertEqual(pick_action.grasp, Grasp.FRONT)

    def test_partial_pickup_action_insert_param(self):
        milk_desig = BelieveObject(names=["milk"])
        pick = PickUpAction(milk_desig, [Arms.LEFT, Arms.RIGHT])
        pick_action = pick(grasp=Grasp.FRONT).resolve()
        self.assertEqual(pick_action.grasp, Grasp.FRONT)