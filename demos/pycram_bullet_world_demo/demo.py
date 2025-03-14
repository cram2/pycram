from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycrap.ontologies import Robot, Apartment, Milk, Cereal, Spoon, Bowl
import numpy as np

np.random.seed(420)
extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.GUI)

robot = Object("pr2", Robot, f"pr2{extension}", pose=Pose([1, 2, 0]))
apartment = Object("apartment", Apartment, f"apartment{extension}")

milk = Object("milk", Milk, "milk.stl", pose=Pose([2.5, 2, 1.02]),
              color=Color(1, 0, 0, 1))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
                pose=Pose([2.5, 2.3, 1.05]), color=Color(0, 1, 0, 1))
spoon = Object("spoon", Spoon, "spoon.stl", pose=Pose([2.4, 2.24, 0.85]),
               color=Color(0, 0, 1, 1))
bowl = Object("bowl", Bowl, "bowl.stl", pose=Pose([2.5, 2.2, 1.02]),
              color=Color(1, 1, 0, 1))
apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])


@with_simulated_robot
def move_and_detect(obj_type):
    NavigateActionDescription(target_location=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtActionDescription(target=[pick_pose]).resolve().perform()

    object_desig = DetectActionDescription(technique=DetectionTechnique.TYPES,
                                           object_designator_description=BelieveObject(
                                               types=[obj_type])).resolve().perform()
    return object_desig[0]


with simulated_robot:
    ParkArmsActionDescription([Arms.BOTH]).resolve().perform()

    MoveTorsoActionDescription([TorsoState.HIGH]).resolve().perform()

    milk_desig = move_and_detect(Milk)

    TransportActionDescription(milk_desig, [Pose([4.8, 3.55, 0.8])], [Arms.LEFT]).resolve().perform()

    cereal_desig = move_and_detect(Cereal)

    TransportActionDescription(cereal_desig, [Pose([5.2, 3.4, 0.8], [0, 0, 1, 1])], [Arms.RIGHT]).resolve().perform()

    bowl_desig = move_and_detect(Bowl)

    TransportActionDescription(bowl_desig, [Pose([5, 3.3, 0.8], [0, 0, 1, 1])], [Arms.LEFT]).resolve().perform()

    MoveTorsoActionDescription([TorsoState.HIGH]).resolve().perform()
    # Finding and navigating to the drawer holding the spoon
    handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig)
    drawer_open_loc = AccessingLocation(handle_desig=handle_desig,
                                        robot_desig=robot_desig,
                                        arm=Arms.RIGHT)

    NavigateActionDescription(drawer_open_loc).resolve().perform()

    OpenActionDescription(object_designator_description=handle_desig, arm=Arms.RIGHT).resolve().perform()
    spoon.detach(apartment)
    ParkArmsActionDescription(Arms.BOTH).resolve().perform()
    # Detect and pickup the spoon
    LookAtActionDescription([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()

    spoon_desigs = DetectActionDescription(technique=DetectionTechnique.TYPES,
                                           object_designator_description=BelieveObject(
                                               types=[Spoon])).resolve().perform()
    spoon_desig = spoon_desigs[0]
    pickup_arm = Arms.LEFT

    ParkArmsActionDescription([Arms.BOTH]).resolve().perform()
    NavigateActionDescription(CostmapLocation(spoon_desig,
                                              reachable_for=robot_desig.resolve(),
                                              reachable_arm=pickup_arm,
                                              grasps=[Grasp.TOP])).resolve().perform()

    PickUpActionDescription(spoon_desig, [pickup_arm], [Grasp.TOP]).resolve().perform()

    ParkArmsActionDescription([Arms.LEFT if pickup_arm == Arms.LEFT else Arms.RIGHT]).resolve().perform()

    NavigateActionDescription(drawer_open_loc).resolve().perform()

    CloseActionDescription(object_designator_description=handle_desig, arm=Arms.RIGHT).resolve().perform()

    ParkArmsActionDescription(Arms.BOTH).resolve().perform()

    MoveTorsoActionDescription(TorsoState.MID).resolve().perform()

    # Find a pose to place the spoon, move and then place it
    spoon_target_pose = Pose([4.7, 3.25, 0.8], [0, 0, 1, 1])
    MoveTorsoActionDescription([TorsoState.HIGH]).resolve().perform()
    placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve(),
                                  reachable_arm=pickup_arm).resolve()

    NavigateActionDescription(placing_loc).resolve().perform()

    PlaceActionDescription(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

    ParkArmsActionDescription(Arms.BOTH).resolve().perform()

world.exit()
