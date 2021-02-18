import rospy
rospy.init_node('pycram')

from pycram.robot_description import InitializedRobotDescription as robot_description
import available_process_modules
import motion_designators  # Needs to be imported to load Process Modules and designator solutions
import pycram.bullet_world_reasoning as btr
from pycram.designator import MotionDesignator
from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld, Object
from pycram.language import macros, par


world = BulletWorld()
world.set_gravity([0, 0, -9.8])
plane = Object("floor", "environment", "../../resources/plane.urdf", world=world)
robot = Object("boxy", "robot", "../../resources/" + robot_description.i.name + ".urdf")

spawning_poses = {
    'milk': [1.3, 1, 1],
    'spoon': [1.35, 0.7, 0.8],
    'cereal': [1.3, 0.6, 1],
    'bowl': [1.3, 0.8, 1]
}

kitchen = Object("kitchen", "environment", "../../resources/kitchen.urdf")
milk = Object("milk", "milk", "../../resources/milk.stl", spawning_poses["milk"])
spoon = Object("spoon", "spoon", "../../resources/spoon.stl", spawning_poses["spoon"])
kitchen.attach(spoon, link="sink_area_left_upper_drawer_main")
cereal = Object("cereal", "cereal", "../../resources/breakfast_cereal.stl", spawning_poses["cereal"])
bowl = Object("bowl", "bowl", "../../resources/bowl.stl", spawning_poses["bowl"])
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
    'boxy': {'sink': [[0.4, 0.7, 0], [0, 0, 0, 1]],
             'island': [[0.2, 0.8, 0], [0, 0, 1, 0]]},
    'donbot': {'sink': [[0.5, 1.2, 0], [0, 0, 0.7, 0.7]],
               'spoon': [[0.5, 1.4, 0], [0, 0, 0.7, 0.7]],
               'island': [[-0.3, 1.2, 0], [0, 0, -0.7, 0.7]]},
    'hsr': {'hsr_cereal': [[0.2, 1.2, 0], [0, 0, 0.7, 0.7]],
            'kitchen_entry': [[0.2, -2.2, 0], [0, 0, -0.7, 0.7]]}
}


def park_arms(robot_name):
    # Parking description
    park_desc = [('type', 'move-arm-joints'), ('left-arm', 'park')]
    if robot_name is not 'donbot' and robot_name is not 'hsr':
        park_desc.append(('right-arm', 'park'))
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
    ProcessModule.perform(MotionDesignator([('type', 'looking'), ('target', object_type)]))

    # Detect object
    # Try to detect object via camera, if this fails...
    det_obj = ProcessModule.perform(MotionDesignator([('type', 'detecting'), ('object', object_type)]))
    block_new = None
    if det_obj:
        block = btr.blocking(det_obj, BulletWorld.robot, gripper)
        block_new = list(filter(lambda obj: obj.type != "environment", block))
    else:
        # ... the robot grasps the object by using its knowledge of the environment.
        det_obj = ProcessModule.perform(MotionDesignator([('type', "world-state-detecting"),
                                                          ('object', object_type)]))

    # If something is in the way, move it first and then move back to the sink.
    if block_new:
        move_object(block_new[0].type, targets[block_new[0].type][0], arm, robot_name)
        move_robot(robot_name, 'sink')

    if det_obj.type == "spoon":
        kitchen.detach(det_obj)

    # Pick up the object
    ProcessModule.perform(MotionDesignator([('type', 'pick-up'), ('object', det_obj), ('arm', arm)]))

    park_arms(robot_name)

    # Move to island
    move_robot(robot_name, 'island')

    # Look at target (also quickfix for not colliding with kitchen if robot has odom frame :/ )
    ProcessModule.perform(MotionDesignator([('type', 'looking'), ('target', targets[object_type][0])]))

    # Place object if target pose of object is reachable for the robots manipulator
    if btr.reachable_pose(target, robot, gripper, threshold=0.05):
        ProcessModule.perform(
            MotionDesignator([('type', 'place'), ('object', det_obj), ('target', target), ('arm', arm)]))

    park_arms(robot_name)
    print("placed: ", object_type)

    # if not btr.stable(det_obj):
    # raise btr.ReasoningError
    targets[object_type][2] = True


if 'hsr' not in robot_description.i.name:
    object_types = ['milk',
                    'bowl',
                    'cereal',
                    'spoon']
    for i in range(0, 4):
        if not targets[object_types[i]][2]:
            position_target = targets[object_types[i]][0]
            arm = targets[object_types[i]][1] if robot_description.i.name is not 'donbot' else 'left'
            move_object(object_types[i], position_target, arm, robot_description.i.name)
else:
    # Spawn object to be manipulated from hsr
    robot_name = 'hsr'
    hsr_cereal = Object("hsr_cereal", "cereal", "../../resources/breakfast_cereal.stl", [0.2, 1.6, 0.5])
    # Park arms
    park_arms(robot_name)
    # Move to object
    move_robot(robot_name, 'hsr_cereal')
    # Look at object
    ProcessModule.perform(MotionDesignator([('type', 'looking'), ('target', 'down')]))
    # Detect object
    det_obj = ProcessModule.perform(MotionDesignator([('type', 'detecting'), ('object', 'cereal')]))
    # Open Gripper
    ProcessModule.perform(MotionDesignator([('type', 'opening-gripper'), ('gripper', 'left')]))
    # Pick up detected object
    ProcessModule.perform(MotionDesignator([('type', 'pick-up'), ('object', det_obj), ('arm', 'left')]))
    # Close Gripper
    ProcessModule.perform(MotionDesignator([('type', 'closing-gripper'), ('gripper', 'left'), ]))
    # Park Arms
    park_arms(robot_name)
    # Move to kitchen entry
    move_robot(robot_name, 'kitchen_entry')
    # Drop the cereal box
    ProcessModule.perform(
        MotionDesignator([('type', 'place'), ('object', det_obj), ('target', [0.2, -2.5, 0.4]), ('arm', 'left')]))
    # Open Gripper
    ProcessModule.perform(MotionDesignator([('type', 'opening-gripper'), ('gripper', 'left')]))
    # Park Arms
    park_arms(robot_name)
