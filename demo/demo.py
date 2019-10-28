import sys
sys.path.append('/home/jonas/Documents/Studium/BA/pycram/src/')
import process_modules
import motion_designators # Needs o be imported to load Process Modules and designator solutions
import pycram.bullet_world_reasoning as btr
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

object_types = ['milk', 'spoon', 'cereal', 'bowl']


def move_object(object_type, target):

    ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('left-arm', 'park'), ('right-arm', 'park')]))

    ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [0.65, 1, 0])]))

    #ProcessModule.perform(MotionDesignator([('type', 'looking'), ('target', [0, 0, 0])]))

    milk_obj = ProcessModule.perform(MotionDesignator([('type', 'detecting'), ('object', 'milk')]))

    if not btr.reachable(milk_obj, BulletWorld.robot, "l_gripper_tool_frame"):
        ProcessModule.perform(MotionDesignator([('type', 'pick-up'), ('object', milk_obj)]))

    ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('right-arm', 'park')]))

    ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [-0.3, 1, 0]), ('orientation', [0, 0, 1, 0.000001])]))

    ProcessModule.perform(MotionDesignator([('type', 'place'), ('object', milk_obj), ('target', [-0.8, 1, 1]), ('arm', 'left')]))

    ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('left-arm', 'park'), ('right-arm', 'park')]))


