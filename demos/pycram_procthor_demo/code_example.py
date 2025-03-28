import pycrap
from pycram.designators.action_designator import NavigateAction,LookAtAction,ParkArmsAction,MoveTorsoAction,DetectAction,TransportAction
from pycram.datastructures.enums import Arms, DetectionTechnique
from pycram.designators.object_designator import BelieveObject
from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import PoseStamped
from pycram.failures import PerceptionObjectNotFound
from pycram.process_module import simulated_robot, with_simulated_robot

def generic_plan(inWorld):
    world = inWorld
    pick_pose = PoseSteamped.from_list([1, -1.78, 0.55])

    robot_desig = BelieveObject(names=["pr2"])
    apartment_desig = BelieveObject(names=["apartment"])

    @with_simulated_robot
    def move_and_detect(obj_type):
        NavigateAction(target_locations=[PoseSteamped.from_list([2, -1.89, 0])]).resolve().perform()

        LookAtAction(targets=[pick_pose]).resolve().perform()

        object_desig = DetectAction(technique=DetectionTechnique.TYPES, object_designator_description=BelieveObject(types=[obj_type])).resolve().perform()

        if len(object_desig)==0:
            raise PerceptionObjectNotFound

        return object_desig[0]

    with (simulated_robot):
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.25]).resolve().perform()

        milk_desig = move_and_detect(pycrap.Milk)

        TransportAction(milk_desig, [PoseSteamped.from_list([4.8, 3.55, 0.8])], [Arms.LEFT]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

