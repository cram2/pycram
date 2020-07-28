import process_modules
import motion_designators # Needs to be imported to load Process Modules and designator solutions
import pycram.bullet_world_reasoning as btr
from pycram.designator import MotionDesignator
from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld, Object
from pycram.language import macros, par

world = BulletWorld()
world.set_gravity([0, 0, -9.8])
plane = Object("floor", "environment", "../../resources/plane.urdf", world=world)
robot = Object("pr2", "robot", "../../resources/pr2.urdf")
kitchen = Object("kitchen", "environment", "../../resources/kitchen.urdf")
milk = Object("milk", "milk", "../../resources/milk.stl", [1.3, 1, 0.93])
spoon = Object("spoon", "spoon", "../../resources/spoon.stl", [1.35, 0.7, 0.8])
kitchen.attach(spoon, link="sink_area_left_upper_drawer_main")
cereal = Object("cereal", "cereal", "../../resources/breakfast_cereal.stl", [1.3, 0.6, 1])
bowl = Object("bowl", "bowl", "../../resources/bowl.stl", [1.3, 0.8, 1])
BulletWorld.robot = robot
robot.set_joint_state("torso_lift_joint", 0.14)

targets = {
    'milk': [[-0.8, 1, 0.93], "left", False],
    'bowl': [[-0.8, 1.2, 0.9], "left", False],
    'cereal': [[-1, 1, 0.94], "left", False],
    'spoon': [[-0.8, 1.2, 0.94], "left", False]
}

counter_poses = {'milk' : [0.55, 0.45, 0],
                 'bowl' : [0.55, 0.25, 0],
                 'cereal' : [0.55, 0.05, 0],
                 'spoon' : [0.35, 0.44, 0]}

table_poses = {'milk' : [-0.05, 1.55, 0],
               'bowl' : [-0.05, 1.75, 0],
               'cereal' : [-0.25, 1.55, 0],
               'spoon' : [-0.05, 1.75, 0]}

def move_object(object_type, target, arm):
    gripper = "l_gripper_tool_frame" if arm == "left" else "r_gripper_tool_frame"
    with par as s:
        ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('left-arm', 'park'), ('right-arm', 'park')]))
        if arm == "left":
            ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', counter_poses[object_type]), ('orientation', [0, 0, 0, 1])]))
        else:
            ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [0.55, 1.35, 0]), ('orientation', [0, 0, 0, 1])]))

    if object_type == "spoon":
        ProcessModule.perform(MotionDesignator([('type', 'accessing'), ('drawer-joint', 'sink_area_left_upper_drawer_main_joint'), ('drawer-handle', 'sink_area_left_upper_drawer_handle'), ('arm', 'left'), ('distance', 0.26), ('part-of', kitchen)]))

        ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', [0.35, 0.24, 0]), ('orientation', [0, 0, 0, 1])]))

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

    ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', table_poses[object_type]), ('orientation', [0, 0, 1, 0])]))

    if btr.reachable_pose(targets[object_type][0], robot, gripper):
        ProcessModule.perform(MotionDesignator([('type', 'place'), ('object', det_obj), ('target', target), ('arm', arm)]))

    ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints'), ('left-arm', 'park'), ('right-arm', 'park')]))
    print("placed: ", object_type)

    #if not btr.stable(det_obj):
        #raise btr.ReasoningError
    targets[object_type][2] = True


object_types = ['milk', 'bowl', 'cereal', 'spoon']
#object_types  = ['spoon']
for i in range(0, 4):
    if not targets[object_types[i]][2]:
        move_object(object_types[i], targets[object_types[i]][0], targets[object_types[i]][1])
