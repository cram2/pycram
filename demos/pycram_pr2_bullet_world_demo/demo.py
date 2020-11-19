from pycram.robot_description import InitializedRobotDescription as robot_description
import boxy_process_modules
import motion_designators  # Needs to be imported to load Process Modules and designator solutions
import pycram.bullet_world_reasoning as btr
from pycram.designator import MotionDesignator
from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld, Object
from pycram.language import macros, par

world = BulletWorld()
world.set_gravity([0, 0, -9.8])
plane = Object("floor", "environment", "../../resources/plane.urdf", world=world)
# robot = Object("pr2", "robot", "../../resources/pr2.urdf")
robot = Object("boxy", "robot", "../../resources/boxy.urdf")
kitchen = Object("kitchen", "environment", "../../resources/kitchen.urdf")
milk = Object("milk", "milk", "../../resources/milk.stl", [1.3, 1, 1])
spoon = Object("spoon", "spoon", "../../resources/spoon.stl", [1.35, 0.7, 0.8])
kitchen.attach(spoon, link="sink_area_left_upper_drawer_main")
cereal = Object("cereal", "cereal", "../../resources/breakfast_cereal.stl", [1.3, 0.6, 1])
bowl = Object("bowl", "bowl", "../../resources/bowl.stl", [1.3, 0.8, 1])
BulletWorld.robot = robot

targets = {
    'milk': [[-0.8, 1, 0.93], "left", False],
    'bowl': [[-0.8, 1.2, 0.9], "right", False],
    'cereal': [[-1, 1, 0.94], "right", False],
    'spoon': [[-0.8, 1.2, 0.94], "right", False]
}

moving_targets = {
    'pr2': {'sink': [[0.65, 0.7, 0], [0, 0, 0, 1]],
            'island': [[-0.3, 1, 0], [0, 0, 1, 0]]},
    'boxy': {'sink': [[0.65, 0.7, 0], [0, 0, 0, 1]],
             'island': [[-0.3, 1, 0], [0, 0, 1, 0]]},
    'donbot': {'sink': [[0.6, 1, 0], [0, 0, 0.7, 0.7]],
               'island': [[-0.3, 1.2, 0], [0, 0, -0.7, 0.7]]}
}


def park_arms(robot_name):
    # Parking description
    park_desc = [('type', 'move-arm-joints'), ('left-arm', 'park')]
    if robot_name is not 'donbot':
        park_desc.append([('right-arm', 'park')])
    # Perform Parking with MotionDesignator
    ProcessModule.perform(MotionDesignator(park_desc))


def move_robot(robot_name, to):
    ProcessModule.perform(MotionDesignator([('type', 'moving'),
                                            ('target', moving_targets[robot_name][to][0]),
                                            ('orientation', moving_targets[robot_name][to][1])]))


def move_object(object_type, target, arm, robot_name):
    # Get Gripper frame
    gripper = robot_description.i.get_tool_frame(arm)

    # Move to sink
    with par as s:
        park_arms(robot_name)
        move_robot(robot_name, 'sink')

    # Access object if needed
    if object_type == "spoon":
        ProcessModule.perform(MotionDesignator(
            [('type', 'accessing'), ('drawer-joint', 'sink_area_left_upper_drawer_main_joint'),
             ('drawer-handle', 'sink_area_left_upper_drawer_handle'), ('arm', 'left'), ('distance', 0.3),
             ('part-of', kitchen)]))

    # Look at object
    ProcessModule.perform(MotionDesignator([('type', 'looking'), ('target', [1.3, 0.6, 1])]))

    # Detect object
    det_obj = ProcessModule.perform(MotionDesignator([('type', 'detecting'), ('object', object_type)]))
    if det_obj:
        block = btr.blocking(det_obj, BulletWorld.robot, gripper)
        block_new = list(filter(lambda obj: obj.type != "environment", block))
    else:
        block_new = [ProcessModule.perform(MotionDesignator([('type', "world-state-detecting"),
                                                             ('object', object_type)]))]

    if block_new:
        move_object(block_new[0].type, targets[block_new[0].type][0], arm, robot_name)
        move_robot(robot_name, 'sink')

    if det_obj.type == "spoon":
        kitchen.detach(det_obj)

    ProcessModule.perform(MotionDesignator([('type', 'pick-up'), ('object', det_obj), ('arm', arm)]))

    park_arms(robot_name)

    # Move to island
    move_robot(robot_name, 'island')

    # Place object if target pose of object is reachable for the robots manipulator
    if btr.reachable_pose(target, robot, gripper):
        ProcessModule.perform(
            MotionDesignator([('type', 'place'), ('object', det_obj), ('target', target), ('arm', arm)]))

    park_arms(robot_name)
    print("placed: ", object_type)

    # if not btr.stable(det_obj):
    # raise btr.ReasoningError
    targets[object_type][2] = True


object_types = ['milk',
                'bowl',
                'cereal',
                'spoon']
for i in range(0, 4):
    if not targets[object_types[i]][2]:
        position_target = targets[object_types[i]][0]
        arm = targets[object_types[i]][1] if robot_description.i.name is not 'donbot' else 'left'
        move_object(object_types[i], position_target, arm, robot_description.i.name)
