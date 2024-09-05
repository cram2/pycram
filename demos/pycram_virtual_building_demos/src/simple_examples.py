from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import simulated_robot



def navigate_simple_example():
    navigate_pose = Pose([2.7, 2.15, 1])

    with simulated_robot:
        MoveTorsoAction([0.25]).resolve().perform()

        NavigateAction([navigate_pose]).resolve().perform()

