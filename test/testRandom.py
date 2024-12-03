from pycram.datastructures.pose import Pose
from pycram.description import ObjectDescription
from pycram.designators.motion_designator import LookingMotion, MoveTCPMotion
from pycram.process_module import ProcessModuleManager, simulated_robot
from pycram.robot_description import RobotDescription, RobotDescriptionManager
from pycram.datastructures.enums import GripperState, Arms, WorldMode
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
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
from pycram.designators.object_designator import BelieveObject

def run_look_and_move_tcp():
    rm = RobotDescriptionManager()
    rm.load_description('iCub')

    with simulated_robot:
        pm = ProcessModuleManager.get_manager()

        LookingMotion(Pose([-2, -2, 3])).perform()
        time.sleep(2)
        MoveTCPMotion(Pose([-0.2, 0.1, 0.1]), Arms.LEFT).perform()
        MoveJointsMotion(["torso_roll"],[20.0]).perform()

        pm.exit()

def try_world():
    extension: str = ObjectDescription.get_file_extension()
    rdm = RobotDescriptionManager()
    rdm.load_description("pr2")
    world = BulletWorld(mode=WorldMode.DIRECT)
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))

    cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                        pose=Pose([1.3, 0.7, 0.95]))




run_look_and_move_tcp()
