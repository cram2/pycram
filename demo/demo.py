import process_modules
import motion_designators # Needs o be imported to load Process Modules and designator solutions
import pycram.bullet_world_reasoning as btr
from pycram.designator import MotionDesignator
from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld, Object
from pycram.language import macros, par

world = BulletWorld()
world.set_gravity([0, 0, -9.8])
plane = Object("floor", "environment", "../plane.urdf", world=world)
robot = Object("pr2", "robot", "../pr2.urdf")
kitchen = Object("kitchen", "environment", "../kitchen.urdf")
milk = Object("milk", "milk", "../resources/milk.stl", [1.3, 1, 1])
spoon = Object("spoon", "spoon", "../resources/spoon.stl", [1.4, 0.8, 1])
cereal = Object("cereal", "cereal", "../resources/breakfast_cereal.stl", [1.3, 0.6, 1])
bowl = Object("bowl", "bowl", "../resources/bowl.stl", [1.3, 0.8, 1])
BulletWorld.robot = robot


def move_object(object_type, target, arm):
    gripper = "l_gripper_tool_frame" if arm == "left" else "r_gripper_tool_frame"
    with par as s:
        ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('left-arm', 'park'), ('right-arm', 'park')]))
        ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [0.65, 0.7, 0]), ('orientation', [0, 0, 0, 1])]))

    ProcessModule.perform(MotionDesignator([('type', 'looking'), ('target', [1.3, 0.6, 1])]))

    det_obj = ProcessModule.perform(MotionDesignator([('type', 'detecting'), ('object', object_type)]))
    #print(det_obj.name))
    block = btr.blocking(det_obj, BulletWorld.robot, gripper)
    block_new = list(filter(lambda obj: obj.type != "environment", block))

    if not block_new:
        ProcessModule.perform(MotionDesignator([('type', 'pick-up'), ('object', det_obj), ('arm', arm)]))
    else:
        move_object(block_new[0].type, [0, 0, 1], "right")
        ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [0.65, 0.7, 0]), ('orientation', [0, 0, 0, 1])]))
        ProcessModule.perform(MotionDesignator([('type', 'pick-up'), ('object', det_obj), ('arm', arm)]))

    print(det_obj.name)

    ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('right-arm', 'park')]))

    ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [-0.3, 1, 0]), ('orientation', [0, 0, 1, 0.000001])]))

    ProcessModule.perform(MotionDesignator([('type', 'place'), ('object', det_obj), ('target', target), ('arm', arm)]))

    ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('left-arm', 'park'), ('right-arm', 'park')]))


object_types = ['milk', 'bowl', 'cereal', 'spoon']
target_poses = [[-0.8, 1, 1], [-0.8, 1.2, 1], [-0.8, 1.4, 1], []]
arms = ["left", "right", "right", "left"]
for i in range(0, 4):
    move_object(object_types[i], target_poses[i], arms[i])
