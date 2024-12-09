from pycram.designators.action_designator import NavigateAction,LookAtAction,ParkArmsAction,MoveTorsoAction,DetectAction,TransportAction
from pycram.datastructures.enums import Arms
from pycram.designators.object_designator import BelieveObject
from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot

def language_plan_test():
    pick_pose = Pose([1, -1.78, 0.55])
    @with_simulated_robot
    def move_and_detect(obj_type):
        NavigateAction(target_locations=[Pose([2, -1.89, 0])]).resolve().perform()

        LookAtAction(targets=[pick_pose]).resolve().perform()

        object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

        return object_desig

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.25]).resolve().perform()

        milk_desig = move_and_detect(ObjectType.MILK)

        TransportAction(milk_desig, [Arms.LEFT], [Pose([4.8, 3.55, 0.8])]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()