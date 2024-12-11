from IPython.core.display_functions import clear_output

from pycram.failures import IKError
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
def transporting_demo(apartment_name, robot_name):

    robot_desig = BelieveObject(names=[robot_name])
    apartment_desig = BelieveObject(names=[apartment_name])
    robot = robot_desig.resolve().world_object
    apartment = apartment_desig.resolve().world_object

    if robot.name == "iCub":
        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([4.7, 4.6, 0.8]),
                      color=Color(1, 0, 0, 1))
        milk_target_pose = Pose([4.8, 3.45, 0.8])

        cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                        pose=Pose([4.65, 4.75, 0.8]), color=Color(0, 1, 0, 1))
        cereal_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])

        bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([4.7, 4, 0.75], [0, 0, -1, 1]),
                      color=Color(1, 1, 0, 1))
        bowl_target_pose = Pose([5, 3.3, 0.75], [0, 0, 0, 1])

        spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([4.7, 4.2, 0.75], [0, 0, -1, 1]),
                       color=Color(0, 0, 1, 1))
        spoon_target_pose = Pose([5.2, 3.3, 0.8], [0, 0, 1, 1])

        pick_pose = Pose([4.7, 4.5, 0.8])
        nav_pose = Pose([4, 4.5, 0])
    else:
        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02], [0, 0, 1, 1]),
                      color=Color(1, 0, 0, 1))
        milk_target_pose = Pose([4.8, 3.55, 0.8])

        cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                        pose=Pose([2.5, 2.5, 1.05]), color=Color(0, 1, 0, 1))
        cereal_target_pose = Pose([5.2, 3.4, 0.8], [0, 0, 1, 1])

        bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.4, 2.2, 0.98]),
                      color=Color(1, 1, 0, 1))
        bowl_target_pose = Pose([5, 3.3, 0.8], [0, 0, 1, 1])

        spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.5, 2.2, 0.85]),
                       color=Color(0, 0, 1, 1))
        spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])

        apartment.attach(spoon, 'cabinet10_drawer_top')

        pick_pose = Pose([2.7, 2.15, 1])
        nav_pose = Pose([1.5, 2, 0])

    @with_simulated_robot
    def move_and_detect(obj_type):
        NavigateAction(target_locations=[nav_pose]).resolve().perform()

        LookAtAction(targets=[pick_pose]).resolve().perform()

        object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

        return object_desig

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

        milk_desig = move_and_detect(ObjectType.MILK)

        TransportAction(milk_desig, [Arms.LEFT], [milk_target_pose]).resolve().perform()
        clear_output()

        cereal_desig = move_and_detect(ObjectType.BREAKFAST_CEREAL)

        if robot.name == "tiago_dual":
            #cereal_target_pose = Pose([5.25, 3.4, 0.85], [0, 0, 0, -1])
            TransportAction(cereal_desig, [Arms.LEFT], [cereal_target_pose]).resolve().perform()
        else:
            TransportAction(cereal_desig, [Arms.LEFT], [cereal_target_pose]).resolve().perform()
        clear_output()

        if robot.name not in ["tiago_dual"]:
            bowl_desig = move_and_detect(ObjectType.BOWL)
            TransportAction(bowl_desig, [Arms.LEFT], [bowl_target_pose]).resolve().perform()

        if robot.name == "iCub":
            spoon_desig = move_and_detect(ObjectType.SPOON)

            TransportAction(spoon_desig, [Arms.LEFT], [spoon_target_pose]).resolve().perform()
            clear_output()
        else:

            # Finding and navigating to the drawer holding the spoon
            handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
            closed_location, opened_location = AccessingLocation(handle_desig=handle_desig.resolve(),
                                                                 robot_desig=robot_desig.resolve()).resolve()

            NavigateAction([closed_location.pose]).resolve().perform()

            OpenAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
                       start_goal_location=[closed_location, opened_location]).resolve().perform()
            spoon.detach(apartment)

            # Detect and pickup the spoon
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
            LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()

            spoon_desig = DetectAction(BelieveObject(types=[ObjectType.SPOON])).resolve().perform()

            if robot.name in {"iai_donbot", "fetch"}:
                ParkArmsAction([Arms.BOTH]).resolve().perform()
                PickUpAction(spoon_desig, [Arms.LEFT], [Grasp.TOP]).resolve().perform()

                ParkArmsAction([Arms.BOTH]).resolve().perform()

                # Find a pose to place the spoon, move and then place it
                placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve(),
                                              reachable_arm=Arms.LEFT,
                                              used_grasps=[Grasp.TOP], object_in_hand=spoon_desig).resolve()

                NavigateAction([placing_loc.pose]).resolve().perform()

                PlaceAction(spoon_desig, [spoon_target_pose], [Arms.LEFT]).resolve().perform()

                ParkArmsAction([Arms.BOTH]).resolve().perform()

                NavigateAction([closed_location.pose]).resolve().perform()

                CloseAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
                            start_goal_location=[opened_location, closed_location]).resolve().perform()

                ParkArmsAction([Arms.BOTH]).resolve().perform()
            else:
                pickup_arm = Arms.LEFT if closed_location.arms[0] == Arms.RIGHT else Arms.RIGHT
                try:
                    PickUpAction(spoon_desig, [pickup_arm], [Grasp.TOP]).resolve().perform()
                except IKError:
                    pickup_loc = CostmapLocation(target=spoon_desig, reachable_for=robot_desig.resolve(),
                                                 reachable_arm=pickup_arm, used_grasps=[Grasp.TOP]).resolve()
                    ParkArmsAction([Arms.BOTH]).resolve().perform()
                    NavigateActionPerformable(pickup_loc.pose).perform()
                    PickUpAction(spoon_desig, [pickup_arm], [Grasp.TOP]).resolve().perform()

                ParkArmsAction([Arms.LEFT if pickup_arm == Arms.LEFT else Arms.RIGHT]).resolve().perform()

                NavigateAction([opened_location.pose]).resolve().perform()

                CloseAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
                            start_goal_location=[opened_location, closed_location]).resolve().perform()

                ParkArmsAction([Arms.BOTH]).resolve().perform()

                MoveTorsoAction([TorsoState.MID]).resolve().perform()

                # Find a pose to place the spoon, move and then place it
                placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve(),
                                              reachable_arm=pickup_arm, used_grasps=[Grasp.TOP],
                                              object_in_hand=spoon_desig).resolve()

                NavigateAction([placing_loc.pose]).resolve().perform()

                PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

                ParkArmsAction([Arms.BOTH]).resolve().perform()

