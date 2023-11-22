import unittest
from pycram.designators.action_designator import *
from pycram.pose import Pose
from pycram.new_language import Sequential

from anytree import RenderTree


class LanguageTestCase(unittest.TestCase):

    def test_seq(self):
        act = NavigateAction([Pose()])
        act2 = MoveTorsoAction([0.3])

        plan = act + act2
        self.assertTrue(isinstance(plan, Sequential))
        print(RenderTree(plan))


if __name__ == '__main__':
    unittest.main()
