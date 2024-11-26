from pycram.datastructures.pose import Pose
from pycram.designators.motion_designator import LookingMotion
from pycram.process_module import ProcessModuleManager, simulated_robot
from pycram.robot_description import RobotDescription, RobotDescriptionManager

rm = RobotDescriptionManager()
rm.load_description('iCub')

with simulated_robot:
    pm = ProcessModuleManager.get_manager()

    LookingMotion(Pose([-2, -2, 3])).perform()

    pm.exit()
