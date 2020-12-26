import os
import process_modules
import motion_designators # Needs to be imported to load Process Modules and designator solutions
import pycram.bullet_world_reasoning as btr
from pycram.designator import MotionDesignator
from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld, Object
from pycram.language import macros, par

resources_path = os.path.join(os.path.dirname(__file__), '..', 'resources')
world = BulletWorld()
world.set_gravity([0, 0, -9.8])
plane = Object("floor", "environment", os.path.join(resources_path, "plane.urdf"), world=world)
robot = Object("pr2", "robot", os.path.join(resources_path, "pr2.urdf"))
kitchen = Object("kitchen", "environment", os.path.join(resources_path, "kitchen.urdf"))
milk = Object("milk", "milk", os.path.join(resources_path, "milk.stl"), [1.3, 1, 1])
spoon = Object("spoon", "spoon", os.path.join(resources_path, "spoon.stl"), [1.35, 0.7, 0.8])
kitchen.attach(spoon, link="sink_area_left_upper_drawer_main")
cereal = Object("cereal", "cereal", os.path.join(resources_path, "breakfast_cereal.stl"), [1.3, 0.6, 1])
bowl = Object("bowl", "bowl", os.path.join(resources_path, "bowl.stl"), [1.3, 0.8, 1])
BulletWorld.robot = robot

targets = {
    'milk': [[-0.8, 1, 0.93], "left", False],
    'bowl': [[-0.8, 1.2, 0.9], "right", False],
    'cereal': [[-1, 1, 0.94], "right", False],
    'spoon': [[-0.8, 1.2, 0.94], "right", False]
}


def move_object(object_type, target, arm):
    gripper = "l_gripper_tool_frame" if arm == "left" else "r_gripper_tool_frame"
    with par as s:
        ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('left-arm', 'park'), ('right-arm', 'park')]))
        ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [0.65, 0.7, 0]), ('orientation', [0, 0, 0, 1])]))

    if object_type == "spoon":
        ProcessModule.perform(MotionDesignator([('type', 'accessing'), ('drawer-joint', 'sink_area_left_upper_drawer_main_joint'), ('drawer-handle', 'sink_area_left_upper_drawer_handle'), ('arm', 'left'), ('distance', 0.3), ('part-of', kitchen)]))


    ProcessModule.perform(MotionDesignator([('type', 'looking'), ('target', [1.3, 0.6, 1])]))

    det_obj = ProcessModule.perform(MotionDesignator([('type', 'detecting'), ('object', object_type)]))

    block = btr.blocking(det_obj, BulletWorld.robot, gripper)
    block_new = list(filter(lambda obj: obj.type != "environment", block))

    if block_new:
        move_object(block_new[0].type, targets[block_new[0].type][0], targets[block_new[0].type][1])
        ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [0.65, 0.7, 0]), ('orientation', [0, 0, 0, 1])]))

    if det_obj.type == "spoon":
        kitchen.detach(det_obj)

    ProcessModule.perform(MotionDesignator([('type', 'pick-up'), ('object', det_obj), ('arm', arm)]))

    ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('left-arm', 'park'), ('right-arm', 'park')]))

    ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [-0.3, 1, 0]), ('orientation', [0, 0, 1, 0])]))

    if btr.reachable_pose(targets[object_type][0], robot, gripper):
        ProcessModule.perform(MotionDesignator([('type', 'place'), ('object', det_obj), ('target', target), ('arm', arm)]))

    ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('left-arm', 'park'), ('right-arm', 'park')]))
    print("placed: ", object_type)

    #if not btr.stable(det_obj):
        #raise btr.ReasoningError
    targets[object_type][2] = True


object_types = ['milk', 'bowl', 'cereal']
for i in range(0, 3):
    if not targets[object_types[i]][2]:
        move_object(object_types[i], targets[object_types[i]][0], targets[object_types[i]][1])
