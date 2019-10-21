import sys
sys.path.append('/home/jonas/Documents/Studium/BA/pycram/src/')
import process_modules
import motion_designators # Needs o be imported to load Process Modules and designator solutions
from pycram.designator import MotionDesignator
from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld, Object

world = BulletWorld()
world.set_gravity([0, 0, -9.8])
plane = Object("floor", "environment", "../plane.urdf", world=world)
robot = Object("pr2", "robot", "../pr2.urdf")
kitchen = Object("kitchen", "environment", "../kitchen.urdf")
milk = Object("milk", "milk", "../resources/milk.stl", [1.3, 1, 1])
spoon = Object("spoon", "spoon", "../resources/spoon.stl", [1.4, 0.8, 0.8])
cereal = Object("cereal", "cereal", "../resources/breakfast_cereal.stl", [1.3, 0.6, 1])
bowl = Object("bowl", "bowl", "../resources/bowl.stl", [1.3, 0.8, 1])
BulletWorld.robot = robot


ProcessModule.perform(MotionDesignator([('type', 'park-arms')]))

ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [0.65, 1, 0])]))

ProcessModule.perform(MotionDesignator([('type', 'pick-up'), ('object', cereal), ('target', cereal.get_pose()), ('arm', 'right')]))

#world.set_realtime(True)
