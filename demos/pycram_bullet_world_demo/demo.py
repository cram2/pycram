import rospy

from pycram.robot_description import InitializedRobotDescription as robot_description
import available_process_modules
import motion_designators  # Needs to be imported to load Process Modules and designator solutions
import pycram.bullet_world_reasoning as btr
from pycram.motionDesignator import *
from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld, Object
from pycram.language import macros, par


world = BulletWorld()
world.set_gravity([0, 0, -9.8])
plane = Object("floor", "environment", "../../resources/plane.urdf", world=world)
robot = Object("boxy", "robot", "../../resources/" + robot_description.i.name + ".urdf")

spawning_poses = {
    'milk': [1.3, 1, 0.93],
    'spoon': [1.35, 0.9, 0.78],
    'cereal': [1.3, 0.6, 0.94],
    'bowl': [1.3, 0.8, 0.94]
}


kitchen = Object("kitchen", "environment", "../../resources/kitchen.urdf")
milk = Object("milk", "milk", "../../resources/milk.stl", spawning_poses["milk"])
spoon = Object("spoon", "spoon", "../../resources/spoon.stl", spawning_poses["spoon"])
if robot_description.i.name == "boxy":
    spoon.set_orientation([0, 0, 1, 0])
kitchen.attach(spoon, link="sink_area_left_upper_drawer_main")
cereal = Object("cereal", "cereal", "../../resources/breakfast_cereal.stl", spawning_poses["cereal"])
bowl = Object("bowl", "bowl", "../../resources/bowl.stl", spawning_poses["bowl"])
BulletWorld.robot = robot
robot.set_joint_state(robot_description.i.torso_joint, 0.24)

targets = {
    'milk': [[-0.8, 1, 0.93], "left", False],
    'bowl': [[-0.8, 1.2, 0.9], "right", False],
    'cereal': [[-1, 1, 0.94], "right", False],
    'spoon': [[-0.8, 1.2, 0.94], "left", False]
}

moving_targets = {
    'pr2' : {'sink' : { 'milk' : [[0.6, 0.4, 0], [0, 0, 0, 1]],
                        'bowl' : [[0.65, 1.4, 0], [0, 0, 0, 1]],
                        'cereal' : [[0.65, 1.3, 0], [0, 0, 0, 1]],
                        'spoon': [[0.4, 0.35, 0], [0, 0, 0, 1]]}, # (0.6, 0.35, 0)
            'island' : { 'milk' : [[-0.3, 1.6, 0], [0, 0, 1, 0]],
                         'bowl' : [[-0.3, 0.5, 0], [0, 0, 1, 0]],
                         'cereal': [[-0.35, 0.4, 0], [0, 0, 1, 0]],
                         'spoon' : [[-0.3, 1.6, 0], [0, 0, 1, 0]]} },
    'boxy' : {'sink' : { 'milk' : [[0.1, 0.7, 0], [0, 0, 0, 1]],
                        'bowl' : [[0.2, 1.5, 0], [0, 0, 0, 1]],
                        'cereal' : [[0.2, 1.3, 0], [0, 0, 0, 1]],
                        'spoon': [[0.05, 0.6, 0], [0, 0, 0, 1]]},
            'island' : { 'milk' : [[0.3, 1.5, 0], [0, 0, 1, 0]],
                         'bowl' : [[0.3, 0.5, 0], [0, 0, 1, 0]],
                         'cereal': [[0.2, 0.8, 0], [0, 0, 1, 0]],
                         'spoon' : [[0.3, 1.7, 0], [0, 0, 1, 0]]} },
#    'pr2': {'sink': [[0.6, 0.4, 0], [0, 0, 0, 1]], #0.65, 0.7, 0
#            'island': [[-0.3, 1.6, 0], [0, 0, 1, 0]]},
#    'boxy': {'sink': [[0.4, 0.7, 0], [0, 0, 0, 1]],
#             'island': [[0.2, 0.8, 0], [0, 0, 1, 0]]},
    'donbot': {'sink': [[0.5, 1.2, 0], [0, 0, 0.7, 0.7]],
               'spoon': [[0.5, 1.4, 0], [0, 0, 0.7, 0.7]],
               'island': [[-0.3, 1.2, 0], [0, 0, -0.7, 0.7]]},
    'hsr': {'hsr_cereal': [[0.2, 1.2, 0], [0, 0, 0.7, 0.7]],
            'kitchen_entry': [[0.2, -2.2, 0], [0, 0, -0.7, 0.7]]}
}


def park_arms(robot_name):
    # Parking description
    park_desc = MoveArmJointsMotionDescription(left_arm_config='park', right_arm_config='park')
    #if robot_name != 'donbot' and robot_name != 'hsr':
    #    park_desc.append(('right-arm', 'park'))
    # Perform Parking with MotionDesignator
    ProcessModule.perform(MotionDesignator(park_desc))

def move_robot(robot_name, to, object):
    ProcessModule.perform(MotionDesignator(MoveMotionDescription(target=moving_targets[robot_name][to][object][0],
                                                orientation=moving_targets[robot_name][to][object][1])))


def move_object(object_type, target, arm, robot_name):
    # Get Gripper frame
    gripper = robot_description.i.get_tool_frame(arm)

    # Move to sink
    with par as s:
        park_arms(robot_name)
        move_robot(robot_name, 'sink', object_type)

    # Access object if needed
    if object_type == "spoon":
        ProcessModule.perform(MotionDesignator(AccessingMotionDescription(drawer_joint='sink_area_left_upper_drawer_main_joint',
                                            drawer_handle='sink_area_left_upper_drawer_handle', arm='left',
                                            distance=0.3, part_of=kitchen)))
        if robot_name == "boxy":
            park_arms("boxy")
            ProcessModule.perform(MotionDesignator(MoveMotionDescription(target=[-0.09, 0.61, 0], orientation=[0,0,0,1])))

    # Look at object
    ProcessModule.perform(MotionDesignator(LookingMotionDescription(target=object_type)))

    # Detect object
    # Try to detect object via camera, if this fails...
    det_obj = ProcessModule.perform(MotionDesignator(DetectingMotionDescription(object_type=object_type)))
    block_new = None
    if det_obj:
        block = btr.blocking(det_obj, BulletWorld.robot, gripper)
        block_new = list(filter(lambda obj: obj.type != "environment", block))
    else:
        # ... the robot grasps the object by using its knowledge of the environment.
        det_obj = ProcessModule.perform(MotionDesignator(WorldStateDetectingMotionDescription(object_type=object_type)))

    # If something is in the way, move it first and then move back to the sink.
    if block_new:
        move_object(block_new[0].type, targets[block_new[0].type][0], arm, robot_name)
        move_robot(robot_name, 'sink', object_type)

    if det_obj.type == "spoon":
        kitchen.detach(det_obj)

    # Pick up the object
    ProcessModule.perform(MotionDesignator(PickUpMotionDescription(object=det_obj, arm=arm)))
    park_arms(robot_name)

    # Move to island
    move_robot(robot_name, 'island', object_type)

    # Look at target (also quickfix for not colliding with kitchen if robot has odom frame :/ )
    ProcessModule.perform(MotionDesignator(LookingMotionDescription(target=targets[object_type][0])))

    # Place object if target pose of object is reachable for the robots manipulator
    if btr.reachable(target, robot, gripper, threshold=0.1):
        ProcessModule.perform(MotionDesignator(PlaceMotionDescription(object=det_obj, target=target, arm=arm)))
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
            arm = targets[object_types[i]][1] if robot_description.i.name != 'donbot' else 'left'
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
    ProcessModule.perform(MotionDesignator(LookingMotionDescription(target='down')))
    # Detect object
    det_obj = ProcessModule.perform(MotionDesignator(DetectingMotionDescription(object_type='cereal')))
    # Open Gripper
    ProcessModule.perform(MotionDesignator(MoveGripperMotionDescription(motion='opening', gripper='left')))
    # Pick up detected object
    ProcessModule.perform(MotionDesignator(PickUpMotionDescription(object=det_obj, arm='left')))
    # Close Gripper
    ProcessModule.perform(MotionDesignator(MoveGripperMotionDescription(motion='closing', gripper='left')))
    # Park Arms
    park_arms(robot_name)
    # Move to kitchen entry
    move_robot(robot_name, 'kitchen_entry')
    # Drop the cereal box
    ProcessModule.perform(MotionDesignator(PlaceMotionDescription(object=det_obj, target=[0.2,-2.5,0.4], arm='left')))
    # Open Gripper
    ProcessModule.perform(MotionDesignator(MoveGripperMotionDescription(motion='opening', gripper='left')))
    # Park Arms
    park_arms(robot_name)
