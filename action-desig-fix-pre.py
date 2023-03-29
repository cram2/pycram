from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.designators.location_designaor import *
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot
from pycram.resolver.plans import Arms


world = BulletWorld()
pr2 = Object("pr2", "robot", "pr2.urdf")
kitchen = Object("kitchen", "environment", "kitchen.urdf")
milk = Object("milk", "milk", "milk.stl", position=[0.5, -0.3, 0.5])

o = ObjectDesignator(ObjectDesignatorDescription(name="milk", type="milk"))
