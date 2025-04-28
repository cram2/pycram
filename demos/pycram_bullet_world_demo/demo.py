from pycram.language import SequentialPlan, ParallelPlan, CodePlan
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.process_module import simulated_robot, with_simulated_robot, ProcessModule
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycrap.ontologies import Robot, Apartment, Milk, Cereal, Spoon, Bowl
import numpy as np

np.random.seed(420)
extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.GUI)
viz = VizMarkerPublisher()

robot = Object("pr2", Robot, f"pr2{extension}", pose=PoseStamped.from_list([1, 2, 0]))
apartment = Object("apartment", Apartment, f"apartment{extension}")

milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([2.5, 2, 1.02], [0, 0, 0, 1]),
              color=Color(1, 0, 0, 1))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
                pose=PoseStamped.from_list([2.45, 2.4, 1.05], [0, 0, 0, 1]), color=Color(0, 1, 0, 1))
spoon = Object("spoon", Spoon, "spoon.stl", pose=PoseStamped.from_list([2.4, 2.24, 0.85]),
               color=Color(0, 0, 1, 1))
bowl = Object("bowl", Bowl, "bowl.stl", pose=PoseStamped.from_list([2.35, 2.2, 0.98]),
              color=Color(1, 1, 0, 1))
apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = PoseStamped.from_list([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])


@with_simulated_robot
def move_and_detect(obj_type):
    NavigateActionDescription(target_location=[PoseStamped.from_list([1.7, 2, 0])]).resolve().perform()

    LookAtActionDescription(target=[pick_pose]).resolve().perform()

    object_desig = DetectActionDescription(technique=DetectionTechnique.TYPES,
                                           object_designator_description=BelieveObject(
                                               types=[obj_type])).resolve().perform()

    return object_desig[0]


with simulated_robot:
    SequentialPlan(

        ParallelPlan(
            ParkArmsActionDescription([Arms.BOTH]),

            MoveTorsoActionDescription([TorsoState.HIGH])),

        CodePlan(loginfo("Handling milk")),

        TransportActionDescription(move_and_detect(Milk), [PoseStamped.from_list([4.8, 3.55, 0.8])], [Arms.LEFT]),

        CodePlan(loginfo("Handling cereal")),

        TransportActionDescription(move_and_detect(Cereal), [PoseStamped.from_list([5.2, 3.4, 0.8], [0, 0, 1, 1])],
                                   [Arms.RIGHT]),

        CodePlan(loginfo("Handling bowl")),

        TransportActionDescription(move_and_detect(Bowl), [PoseStamped.from_list([5, 3.3, 0.8], [0, 0, 1, 1])],
                                   [Arms.LEFT]),

        CodePlan(loginfo("Opening drawer to get spoon")),

        MoveTorsoActionDescription([TorsoState.HIGH]),
        # Finding and navigating to the drawer holding the spoon

        NavigateActionDescription(
            AccessingLocation(handle=ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig),
                              robot_desig=robot_desig,
                              arm=Arms.RIGHT)),

        OpenActionDescription(
            object_designator_description=ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig),
            arm=Arms.RIGHT),
        CodePlan(spoon.detach(apartment)),

        ParkArmsActionDescription(Arms.BOTH),
        # Detect and pickup the spoon
        LookAtActionDescription([apartment.get_link_pose("handle_cab10_t")]),

        CodePlan(loginfo("Detecting and picking up spoon")),

        ParkArmsActionDescription([Arms.BOTH]),
        NavigateActionDescription(CostmapLocation(DetectActionDescription(technique=DetectionTechnique.TYPES,
                                                                          object_designator_description=BelieveObject(
                                                                              types=[Spoon]))[0],
                                                  reachable_for=robot_desig.resolve(),
                                                  reachable_arm=Arms.LEFT,
                                                  grasp_descriptions=GraspDescription(Grasp.FRONT, Grasp.TOP, False))),

        PickUpActionDescription(BelieveObject(types=[Spoon]), [Arms.LEFT],
                                GraspDescription(Grasp.FRONT, Grasp.TOP, False)),

        ParallelPlan(
            ParkArmsActionDescription([Arms.LEFT if Arms.LEFT == Arms.LEFT else Arms.RIGHT]),

            NavigateActionDescription(
                AccessingLocation(handle=ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig),
                                  robot_desig=robot_desig,
                                  arm=Arms.RIGHT))),

        CodePlan(loginfo("Closing drawer")),

        CloseActionDescription(
            object_designator_description=ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig),
            arm=Arms.RIGHT),

        ParallelPlan(
            ParkArmsActionDescription(Arms.BOTH),

            MoveTorsoActionDescription(TorsoState.MID)),
        CodePlan(loginfo("Placing spoon")),

        # Find a pose to place the spoon, move and then place it
        ParallelPlan(
            MoveTorsoActionDescription([TorsoState.HIGH]),

            NavigateActionDescription(CostmapLocation(target=PoseStamped.from_list([4.7, 3.25, 0.8], [0, 0, 1, 1]),
                                                      reachable_for=robot_desig.resolve(),
                                                      reachable_arm=[Arms.LEFT],
                                                      object_in_hand=BelieveObject(types=[Spoon])))),

        PlaceActionDescription(BelieveObject(types=[Spoon]), PoseStamped.from_list([4.7, 3.25, 0.8], [0, 0, 1, 1]),
                               [Arms.LEFT]),

        ParkArmsActionDescription(Arms.BOTH))

viz._stop_publishing()
world.exit()
