import os

from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.process_module import simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


def navigate_main():
    extension = ObjectDescription.get_file_extension()

    world = BulletWorld(WorldMode.DIRECT)
    viz_marker = VizMarkerPublisher()
    robot = Object("hsrb", ObjectType.ROBOT, f"hsrb{extension}", pose=Pose([1, 2, 0]))
    apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment{extension}")

    robot_desig = BelieveObject(names=["hsrb"])
    apartment_desig = BelieveObject(names=["apartment"])

    navigate_pose = Pose([2.7, 2.15, 1])

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.25]).resolve().perform()

        NavigateAction([navigate_pose]).resolve().perform()


def navigate_demo(s=None, w=None, f=None):
    # launch stuff
    navigate_main()


if __name__ == "__main__":
    navigate_main()

