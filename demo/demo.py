import sys
sys.path.append('/home/jonas/Documents/Studium/BA/pycram/src/')
import process_modules
import motion_designators
import pybullet as p
import time
from pycram.designator import MotionDesignator
from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld, Object

world = BulletWorld()
robot = Object("pr2", "../pr2.urdf")
#kitchen = Object("kitchen", "../kitchen.urdf")
denkmit = Object("denkmit", "../denkmit.obj", [0.3, 0, 1])
BulletWorld.robot = robot

ProcessModule.perform(MotionDesignator([('type', 'pick-up'), ('object', denkmit), ('target', denkmit.get_pose())]))
ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [1, 0, 0])]))
time.sleep(2)
world.simulate(10)

