import unittest
from pycram.designators import action_designator
import pycram.enums


class TestActionDesignatorGrounding(unittest.TestCase):
    """Testcase for the grounding methods of action designators."""

    def test_move_torso(self):
        description = action_designator.MoveTorsoAction([0.3])
        self.assertEqual(description.ground().position, 0.3)

    def test_set_gripper(self):
        description = action_designator.SetGripperAction(["left"], [True, False])
        self.assertEqual(description.ground().gripper, "left")
        self.assertEqual(description.ground().opening, True)
        self.assertEqual(len(list(iter(description))), 2)

    def test_release(self):
        description = action_designator.ReleaseAction(["left"], [1])
        self.assertEqual(description.ground().gripper, "left")
        self.assertEqual(description.ground().object_designator, 1)

    def test_grip(self):
        description = action_designator.GripAction(["left"], [1], [0.5])
        self.assertEqual(description.ground().gripper, "left")
        self.assertEqual(description.ground().object_designator, 1)

    def test_park_arms(self):
        description = action_designator.ParkArmsAction([pycram.enums.Arms.BOTH])
        self.assertEqual(description.ground().arm, pycram.enums.Arms.BOTH)

    def test_pick_up(self):
        description = action_designator.PickUpAction([0], ["left"], ["front"])
        # TODO: test something

    def test_place(self):
        description = action_designator.PlaceAction(0, [([0, 0, 0], [0, 0, 0, 1])], ["left"])
        # TODO: test something

    def test_navigate(self):
        description = action_designator.NavigateAction([([0, 0, 0], [0, 0, 0, 1])])
        self.assertEqual(description.ground().target_location, ([0, 0, 0], [0, 0, 0, 1]))

    def test_transport(self):
        description = action_designator.TransportAction(0, ["left"], [([0, 0, 0], [0, 0, 0, 1])])
        # TODO: test something

    def test_look_at(self):
        description = action_designator.LookAtAction([[0, 0, 0]])
        self.assertEqual(description.ground().target, [0, 0, 0])

    def test_detect(self):
        description = action_designator.DetectAction(9)
        # TODO: test something

    def test_open(self):
        description = action_designator.OpenAction(0, ["left"], [1])
        # TODO: test something

    def test_close(self):
        description = action_designator.CloseAction(0, ["left"])
        # TODO: test something


if __name__ == '__main__':
    unittest.main()
