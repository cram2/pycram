from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot

world = BulletWorld()
robot = Object("pr2", "robot", "pr2.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", "environment", "apartment.urdf")

milk = Object("milk", "milk", "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]), color=[0, 1, 0, 1])
spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 1])
apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])

@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig


with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.25]).resolve().perform()

    milk_desig = move_and_detect("milk")

    TransportAction(milk_desig, ["left"], [Pose([4.8, 3.55, 0.8])]).resolve().perform()

    cereal_desig = move_and_detect("cereal")

    TransportAction(cereal_desig, ["right"], [Pose([5.2, 3.4, 0.8], [0, 0, 1, 1])]).resolve().perform()

    bowl_desig = move_and_detect("bowl")

    TransportAction(bowl_desig, ["left"], [Pose([5, 3.3, 0.8], [0, 0, 1, 1])]).resolve().perform()

    # Finding and navigating to the drawer holding the spoon
    handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
    drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(), robot_desig=robot_desig.resolve()).resolve()

    NavigateAction([drawer_open_loc.pose]).resolve().perform()

    OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
    spoon.detach(apartment)

    # Detect and pickup the spoon
    LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()

    spoon_desig = DetectAction(BelieveObject(types=["spoon"])).resolve().perform()

    pickup_arm = "left" if drawer_open_loc.arms[0] == "right" else "right"
    PickUpAction(spoon_desig, [pickup_arm], ["top"]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.15]).resolve().perform()

    # Find a pose to place the spoon, move and then place it
    spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])
    placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve()).resolve()

    NavigateAction([placing_loc.pose]).resolve().perform()

    PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

