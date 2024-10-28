from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, Arms
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import ParkArmsAction, MoveTorsoAction, TransportAction, NavigateAction, \
    LookAtAction, DetectAction
from pycram.designators.object_designator import BelieveObject
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.world_concepts.world_object import Object
from pycram.worlds.multiverse import Multiverse


@with_simulated_robot
def move_and_detect(obj_type: ObjectType, pick_pose: Pose):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig

world = Multiverse()
robot = Object('pr2', ObjectType.ROBOT, f'pr2.urdf', pose=Pose([1.3, 2, 0.01]))
apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment.urdf")

milk = Object("milk", ObjectType.MILK, f"milk.stl", pose=Pose([2.4, 2, 1.02]),
              color=Color(1, 0, 0, 1))

robot_desig = BelieveObject(names=[robot.name])
apartment_desig = BelieveObject(names=[apartment.name])

with simulated_robot:

    # Transport the milk
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.25]).resolve().perform()

    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[Pose([2.6, 2.15, 1])]).resolve().perform()

    milk_desig = DetectAction(BelieveObject(types=[milk.obj_type])).resolve().perform()

    TransportAction(milk_desig, [Arms.LEFT], [Pose([2.4, 3, 1.02])]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

world.exit()
