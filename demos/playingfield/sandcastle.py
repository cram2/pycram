from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color

def param_plan(robot_path,environment_path):
    world = BulletWorld(WorldMode.GUI)
    robot = Object("pr2", ObjectType.ROBOT, robot_path, pose=Pose([1, 2, 0]))
    apartment = Object(environment_path[:environment_path.find(".")], ObjectType.ENVIRONMENT, environment_path)

    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1, -1.78, 0.55]),
                  color=Color(1, 0, 0, 1))

    pick_pose = Pose([1, -1.78, 0.55])

    robot_desig = BelieveObject(names=["pr2"])
    apartment_desig = BelieveObject(names=["apartment"])


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
