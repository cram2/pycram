import unittest
from pycram.designators import action_designator


class TestActionDesignatorGrounding(unittest.TestCase):
    """Testcase for the grounding methods of action designators."""

    def test_move_torso(self):
        action = action_designator.MoveTorsoAction(0.3)
        action.ground()


if __name__ == '__main__':
    unittest.main()
