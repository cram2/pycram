from IPython.core.display_functions import clear_output

from pycram.ros.viz_marker_publisher import VizMarkerPublisher, AxisMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color

# extension = ObjectDescription.get_file_extension()

# world = BulletWorld(WorldMode.DIRECT)
# viz = VizMarkerPublisher()

# robot = Object("rollin_justin", ObjectType.ROBOT, f"rollin_justin{extension}", pose=Pose([1, 2, 0]))
# robot = Object("iCub", ObjectType.ROBOT, f"iCub{extension}", pose=Pose([1, 2, 0]))
# robot = Object("slr", ObjectType.ROBOT, f"slr{extension}", pose=Pose([1, 2, 0]))


# apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment-small{extension}")
def cleanup_demo(apartment, robot):
    # milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([4.8, 3.55, 0.8]),
    #               color=Color(1, 0, 0, 1))
    # cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
    #                 pose=Pose([5.2, 3.4, 0.8]), color=Color(0, 1, 0, 1))
    # bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([5, 3.3, 0.8]),
    #               color=Color(1, 1, 0, 1))
    # spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([4.85, 3.3, 0.8]),
    #                color=Color(0, 0, 1, 1))
    #

    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([4.9, 4.2, 0.8]),
                  color=Color(1, 0, 0, 1))
    cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                    pose=Pose([4.8, 3.8, 0.8]), color=Color(0, 1, 0, 1))
    bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([4.7, 4.0, 0.8]),
                  color=Color(1, 1, 0, 1))
    spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([4.7, 4.15, 0.8]),
                   color=Color(0, 0, 1, 1))

    pick_pose = Pose([4.8, 4, 0.8])

    robot_desig = BelieveObject(names=[robot.name])
    apartment_desig = BelieveObject(names=[apartment.name])


    @with_simulated_robot
    def move_and_detect(obj_type):
        NavigateAction(target_locations=[Pose([3.7, 4, 0], [0, 0, 0, 1])]).resolve().perform()

        LookAtAction(targets=[pick_pose]).resolve().perform()

        object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

        return object_desig

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

        milk_desig = move_and_detect(ObjectType.MILK)

        TransportAction(milk_desig, [Arms.LEFT], [Pose([2.5, 2, 1.02], [0, 0, 0, 1])]).resolve().perform()
        clear_output()

        cereal_desig = move_and_detect(ObjectType.BREAKFAST_CEREAL)

        TransportAction(cereal_desig, [Arms.LEFT], [Pose([2.4, 2.2, 1.05], [0, 0, 0, 1])]).resolve().perform()
        clear_output()

        bowl_desig = move_and_detect(ObjectType.BOWL)

        TransportAction(bowl_desig, [Arms.LEFT], [Pose([2.45, 3.3, 0.97], [0, 0, 0, 1])]).resolve().perform()
        clear_output()

        spoon_desig = move_and_detect(ObjectType.SPOON)
        if robot.name == "tiago_dual" or robot.name == "iai_donbot":
            TransportAction(spoon_desig, [Arms.LEFT], [Pose([2.45, 3.5, 0.97], [0, 0, 0, 1])]).resolve().perform()
        else:
            TransportAction(spoon_desig, [Arms.RIGHT], [Pose([2.5, 3.5, 0.97], [0, 0, 0, 1])]).resolve().perform()
