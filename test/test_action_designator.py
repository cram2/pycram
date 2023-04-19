import unittest
from pycram.designators import action_designator, object_designator
import pycram.enums
import test_bullet_world


class TestActionDesignatorGrounding(test_bullet_world.BulletWorldTest):
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
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.ReleaseAction(["left"], object_description)
        self.assertEqual(description.ground().gripper, "left")
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_grip(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.GripAction(["left"], object_description, [0.5])
        self.assertEqual(description.ground().gripper, "left")
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_park_arms(self):
        description = action_designator.ParkArmsAction([pycram.enums.Arms.BOTH])
        self.assertEqual(description.ground().arm, pycram.enums.Arms.BOTH)

    def test_pick_up(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PickUpAction(object_description, ["left"], ["front"])
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_place(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [([0, 0, 0], [0, 0, 0, 1])], ["left"])
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_navigate(self):
        description = action_designator.NavigateAction([([0, 0, 0], [0, 0, 0, 1])])
        self.assertEqual(description.ground().target_location, ([0, 0, 0], [0, 0, 0, 1]))

    def test_transport(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.TransportAction(object_description, ["left"], [([0, 0, 0], [0, 0, 0, 1])])
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_look_at(self):
        description = action_designator.LookAtAction([[0, 0, 0]])
        self.assertEqual(description.ground().target, [0, 0, 0])

    def test_detect(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.DetectAction(object_description)
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_open(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.OpenAction(object_description, ["left"], [1])
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_close(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.CloseAction(object_description, ["left"])
        self.assertEqual(description.ground().object_designator.name, "milk")


if __name__ == '__main__':
    unittest.main()
