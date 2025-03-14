from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot, ProcessModule
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycrap.ontologies import Robot, Apartment, Milk, Cereal, Spoon, Bowl
import numpy as np

np.random.seed(420)
extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.DIRECT)
viz = VizMarkerPublisher()

robot = Object("pr2", Robot, f"pr2{extension}", pose=Pose([1, 2, 0]))
apartment = Object("apartment", Apartment, f"apartment{extension}")

milk = Object("milk", Milk, "milk.stl", pose=Pose([2.5, 2, 1.02], [0, 0, 0, 1]),
              color=Color(1, 0, 0, 1))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
                pose=Pose([2.45, 2.4, 1.05], [0, 0, 0, 1]), color=Color(0, 1, 0, 1))
spoon = Object("spoon", Spoon, "spoon.stl", pose=Pose([2.4, 2.24, 0.85]),
               color=Color(0, 0, 1, 1))
bowl = Object("bowl", Bowl, "bowl.stl", pose=Pose([2.4, 2.2, 0.98]),
              color=Color(1, 1, 0, 1))
apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])


@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(technique=DetectionTechnique.TYPES,
                                object_designator_description=BelieveObject(types=[obj_type])).resolve().perform()
    return object_desig[0]


with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

    loginfo("Handling milk")

    milk_desig = move_and_detect(Milk)

    TransportAction(milk_desig, [Pose([4.8, 3.55, 0.8])], [Arms.LEFT]).resolve().perform()

    loginfo("Handling cereal")

    cereal_desig = move_and_detect(Cereal)

    TransportAction(cereal_desig, [Pose([5.2, 3.4, 0.8], [0, 0, 1, 1])], [Arms.RIGHT]).resolve().perform()

    loginfo("Handling bowl")

    bowl_desig = move_and_detect(Bowl)

    TransportAction(bowl_desig, [Pose([5, 3.3, 0.8], [0, 0, 1, 1])], [Arms.LEFT]).resolve().perform()

    loginfo("Opening drawer to get spoon")

    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    # Finding and navigating to the drawer holding the spoon
    handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
    drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                        robot_desig=robot_desig.resolve()).resolve()

    NavigateAction([drawer_open_loc.pose]).resolve().perform()

    OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
    spoon.detach(apartment)
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    # Detect and pickup the spoon
    LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()

    loginfo("Detecting and picking up spoon")

    spoon_desigs = DetectAction(technique=DetectionTechnique.TYPES,
                                object_designator_description=BelieveObject(types=[Spoon])).resolve().perform()
    spoon_desig = spoon_desigs[0]
    pickup_arm = Arms.LEFT if drawer_open_loc.arms[0] == Arms.RIGHT else Arms.RIGHT

    ParkArmsAction([Arms.BOTH]).resolve().perform()
    top_grasp = GraspDescription(Grasp.FRONT, Grasp.TOP, False)
    spoon_pick_loc = CostmapLocation(spoon_desig, reachable_for=robot_desig.resolve(), reachable_arms=[pickup_arm],
                                     grasp_descriptions=[top_grasp]).resolve()
    NavigateAction([spoon_pick_loc.pose]).resolve().perform()

    PickUpAction(spoon_desig, [pickup_arm], [top_grasp]).resolve().perform()

    ParkArmsAction([Arms.LEFT if pickup_arm == Arms.LEFT else Arms.RIGHT]).resolve().perform()

    NavigateAction([drawer_open_loc.pose]).resolve().perform()

    loginfo("Closing drawer")

    CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([TorsoState.MID]).resolve().perform()

    loginfo("Placing spoon")

    # Find a pose to place the spoon, move and then place it
    spoon_target_pose = Pose([4.7, 3.25, 0.8], [0, 0, 1, 1])
    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve(),
                                  reachable_arms=[pickup_arm], object_in_hand=spoon_desig).resolve()

    NavigateAction([placing_loc.pose]).resolve().perform()

    PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

viz._stop_publishing()
world.exit()
