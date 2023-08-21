from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot

world = BulletWorld()
robot = Object("pr2", "robot", "pr2.urdf", pose=Pose([2, 1, 0]))
apartment = Object("apartment", "environment", "apartment.urdf")

milk = Object("milk", "milk", "milk.stl", pose=Pose([3, 2, 1.02], [0, 0, 1, 0]))
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([3, 2.3, 1.05], [0, 0, 1, 0]))
spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([2.2, 2, 0.85], [0, 0, 0, 1]))
apartment.attach(spoon, 'cabinet10_drawer_top')

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])
milk_desig = BelieveObject(names=["cereal"])
cereal_desig = BelieveObject(names=["milk"])
spoon_desig = BelieveObject(names=["spoon"])


@with_simulated_robot
def move_object():
    pass


with simulated_robot:
    TransportAction(milk_desig, ["left"], [Pose([5, 3.8, 1])]).resolve().perform()

