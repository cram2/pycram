from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, Arms
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import ParkArmsAction, MoveTorsoAction, TransportAction, NavigateAction, \
    LookAtAction, DetectAction
from pycram.designators.object_designator import BelieveObject
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.world_concepts.world_object import Object
from pycram.worlds.multiverse import Multiverse


@with_simulated_robot
def move_and_detect(obj_type: ObjectType, pick_pose: Pose):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig


world = Multiverse(simulation='pycram_test')
extension = ObjectDescription.get_file_extension()
robot = Object('pr2', ObjectType.ROBOT, f'pr2{extension}', pose=Pose([1.3, 2, 0.01]))
apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment{extension}")

milk = Object("pycram_milk", ObjectType.MILK, f"pycram_milk{extension}", pose=Pose([2.4, 2, 1.02]),
              color=Color(1, 0, 0, 1))

pick_pose = Pose([2.6, 2.15, 1])

with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.25]).resolve().perform()

    milk_desig = move_and_detect(ObjectType.MILK, pick_pose)

    TransportAction(milk_desig, [Arms.LEFT], [Pose([2.4, 3, 1.02])]).resolve().perform()

world.reset_world_and_remove_objects()
world.exit()
