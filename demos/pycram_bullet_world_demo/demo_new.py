from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot

world = BulletWorld()
robot = Object("pr2", "robot", "pr2.urdf", pose=Pose([2, 1, 0]))
apartment = Object("apartment", "environment", "apartment.urdf")

milk = Object("milk", "milk", "milk.stl", pose=Pose([3.05, 2, 1.02], [0, 0, 1, 0]))
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([3.05, 2.3, 1.05], [0, 0, 1, 0]))
spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([2.2, 2, 0.85], [0, 0, 0, 1]))
bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([3.05, 2.2, 1.02], [0, 0, 1, 0]))
apartment.attach(spoon, 'cabinet10_drawer_top')

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])
milk_desig = BelieveObject(names=["cereal"])
cereal_desig = BelieveObject(names=["milk"])
spoon_desig = BelieveObject(names=["spoon"])
bowl_desig = BelieveObject(names=["bowl"])


with simulated_robot:
    MoveTorsoAction([0.25]).resolve().perform()

    TransportAction(milk_desig, ["left"], [Pose([5, 3.8, 1])]).resolve().perform()

    TransportAction(cereal_desig, ["right"], [Pose([4.9, 3.7, 1])]).resolve().perform()

    TransportAction(bowl_desig, ["left"], [Pose([5, 4, 1])]).resolve().perform()

    handle_desig = ObjectPart(names=["cabinet10_drawer_top"], part_of=apartment_desig.resolve()).resolve()
    drawer_open_loc = AccessingLocation(handle_desig=handle_desig, robot_desig=robot_desig.resolve()).resolve()

    NavigateAction([drawer_open_loc]).resolve().perform()

    OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

    PickUpAction(spoon_desig, ["right"], ["top"]).resolve().perform()

    placing_loc = CostmapLocation(target=Pose([5.1, 4, 1], [0, 0, 1, 1])).resolve()

    NavigateAction([placing_loc]).resolve().perform()

    PlaceAction(spoon_desig, [Pose([5.1, 4, 1]), [0, 0, 1, 1]], ["right"]).resolve().perform()

