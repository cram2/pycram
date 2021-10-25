from pycram.object_designator import ObjectDesignator
from pycram.task import with_tree
from pycram.process_module import ProcessModule
from pycram.motion_designator import *
from pycram.action_designator import *
from pycram.plan_failures import PlanFailure
#from pr2_knowledge import reach_position_generator, object_fetching_location_generator, free_arms, Arms
from enum import Enum, auto

from pycram.bullet_world import BulletWorld, filter_contact_points


def free_arms():
    if not OBJECT_HELD_LEFT and not OBJECT_HELD_RIGHT:
        return [Arms.RIGHT, Arms.LEFT]
    elif not OBJECT_HELD_LEFT:
        return [Arms.LEFT]
    elif not OBJECT_HELD_RIGHT:
        return [Arms.RIGHT]
    else:
        return []

def reach_position_generator(target):
    if type(target) is ObjectDesignator:
        if target.prop_value('name') in ['sink_area_left_middle_drawer', 'sink_area_left_upper_drawer']:
            yield [0.3, 0.9, 0], [0,0,0,1]
            yield [0.4, 0.9, 0], [0,0,0,1]
            yield [0.5, 0.9, 0], [0,0,0,1]
            yield [0.6, 0.9, 0], [0,0,0,1]
        elif target.prop_value('name') == 'iai_fridge':
            yield [0.5, -0.4, 0], [0, 0, -0.258819, 0.9659258]
        else:
            yield [0.6, 0.9, 0], [0,0,0,1]
    else:
        yield [-1.8, 1, 0], [0,0,0,1]
        yield [-0.4, 1, 0], [0,0,1,0]


def object_fetching_location_generator(object_designator):
    object_type = object_designator.prop_value('type')
    if object_type == "spoon":
        yield ObjectDesignator([('type', 'drawer'), ('name', 'sink_area_left_upper_drawer'), ('part-of', "kitchen")])
    elif object_type == "bowl":
        yield ObjectDesignator([('type', 'drawer'), ('name', 'sink_area_left_middle_drawer'), ('part-of', "kitchen")])
    elif object_type == "milk":
        yield ObjectDesignator([('type', 'fridge'), ('name', 'iai_fridge'), ('part-of', "kitchen")])
        yield [1.3, 0.8, 0.95]  # Location on counter top
    elif object_type == "cereal":
        yield [1.3, 0.8, 0.95]  # Location on counter top
    else:
        # Otherwise just look everywhere
        yield [1.3, 0.8, 0.95]
        yield ObjectDesignator([('type', 'drawer'), ('name', 'sink_area_left_upper_drawer'), ('part-of', "kitchen")])
        yield ObjectDesignator([('type', 'drawer'), ('name', 'sink_area_left_middle_drawer'), ('part-of', "kitchen")])
        yield ObjectDesignator([('type', 'fridge'), ('name', 'iai_fridge'), ('part-of', "kitchen")])


class Arms(Enum):
    LEFT = auto()
    RIGHT = auto()
    BOTH = auto()



@with_tree
def open_gripper(gripper):
    print("Opening gripper {}".format(gripper))
    ProcessModule.perform(MotionDesignator([('type', 'opening-gripper'), ('gripper', gripper)]))

@with_tree
def close_gripper(gripper):
    print("Closing gripper {}.".format(gripper))
    ProcessModule.perform(MotionDesignator([('type', 'closing-gripper'), ('gripper', gripper)]))

@with_tree
def pick_up(arm, btr_object):
    print("Picking up {} with {}.".format(btr_object, arm))
    motion_arm = "left" if arm is Arms.LEFT else "right"
    # TODO: Hack to detach from kitchen.. (Should go into process module maybe)
    try:
        btr_object.prop_value('bullet_obj').detach(BulletWorld.current_bullet_world.get_objects_by_name("kitchen")[0])
    except KeyError:
        print("Not attached to anything!")
    ProcessModule.perform(MotionDesignator([('type', 'pick-up'), ('object', btr_object), ('arm', motion_arm)]))
    # ActionDesignator(ParkArmsDescription(arm=arm)).perform()

@with_tree
def place(arm, btr_object, target):
    print("Placing {} with {} at {}.".format(btr_object, arm, target))
    motion_arm = "left" if arm is Arms.LEFT else "right"
    ProcessModule.perform(MotionDesignator([('type', 'place'), ('object', btr_object), ('arm', motion_arm), ('target', target)]))
    if filter_contact_points(btr_object.prop_value("bullet_obj").contact_points_simulated(), [0,1,2]):
        raise PlanFailure()

@with_tree
def navigate(target, orientation=[0, 0, 0, 1]):
    print("Moving to {}. Orientation: {}.".format(target, orientation))
    ProcessModule.perform(MotionDesignator([('type', 'moving'), ('target', target), ('orientation', orientation)]))

@with_tree
def park_arms(arms):
    print("Parking arms {}.".format(arms))
    left_arm = [('left-arm', 'park')] if arms in [Arms.LEFT, Arms.BOTH] else []
    right_arm = [('right-arm', 'park')] if arms in [Arms.RIGHT, Arms.BOTH] else []
    ProcessModule.perform(MotionDesignator([('type', 'move-arm-joints')] + left_arm + right_arm))

@with_tree
def detect(object_designator):
    print("Detecting object of type {}.".format(object_designator.prop_value("type")))
    det_object =  ProcessModule.perform(MotionDesignator([('type', 'detecting'), ('object', object_designator.prop_value("type"))]))
    if det_object is None:
        raise PlanFailure("No object detected.")
    detected_obj_desig = object_designator.copy([("pose", det_object.get_pose()), ("bullet_obj", det_object)])
    detected_obj_desig.equate(object_designator)
    return detected_obj_desig

@with_tree
def look_at(target):
    print("Looking at {}.".format(target))
    ProcessModule.perform(MotionDesignator([('type', 'looking'), ('target', target)]))

@with_tree
def transport(object_designator, arm, target_location):
    ## Setup TODO: Put this into resolution?
    fetch_object_location_generator = object_fetching_location_generator(object_designator)
    fetch_object_location = next(fetch_object_location_generator)
    if type(fetch_object_location) is ObjectDesignator:
        fetch_robot_position_generator = reach_position_generator(fetch_object_location)
    else:
        fetch_robot_position_generator = reach_position_generator(object_designator)
    pos, rot = next(fetch_robot_position_generator)

    ## Fetch
    # Navigate
    ActionDesignator(ParkArmsDescription(Arms.BOTH)).perform()
    ActionDesignator(NavigateDescription(target_position=pos, target_orientation=rot)).perform()

    # Access
    if type(fetch_object_location) is ObjectDesignator:
        ActionDesignator(OpenActionDescription(fetch_object_location, arm)).perform()
        ActionDesignator(ParkArmsDescription(arm)).perform()

    # Pick Up
    ActionDesignator(LookAtActionDescription(fetch_object_location)).perform()
    obj = ActionDesignator(DetectActionDescription(object_designator)).perform()
    ActionDesignator(PickUpDescription(obj, arm=arm)).perform()
    ActionDesignator(ParkArmsDescription(arm)).perform()

    # Seal
    if type(fetch_object_location) is ObjectDesignator:
        arms_free = free_arms()
        if arms_free:
            ActionDesignator(CloseActionDescription(fetch_object_location, arms_free[0])).perform()
            ActionDesignator(ParkArmsDescription(arms_free[0])).perform()

    ## Deliver
    # Navigate
    deliver_robot_position_generator = reach_position_generator(target_location)
    pos, rot = next(deliver_robot_position_generator) #[-1.8, 1, 0], [0,0,0,1]
    ActionDesignator(NavigateDescription(target_position=pos, target_orientation=rot)).perform()

    # Place
    ActionDesignator(PlaceDescription(obj, target_location=target_location, arm=arm)).perform()
    ActionDesignator(ParkArmsDescription(arm)).perform()


def get_container_joint_and_handle(container_desig):
    name = container_desig.prop_value('name')
    if name == "iai_fridge":
        return "iai_fridge_door_joint", "iai_fridge_door_handle"
    elif name == "sink_area_left_upper_drawer":
        return "sink_area_left_upper_drawer_main_joint", "sink_area_left_upper_drawer_handle"
    elif name == "sink_area_left_middle_drawer":
        return "sink_area_left_middle_drawer_main_joint", "sink_area_left_middle_drawer_handle"
    else:
        raise NotImplementedError()

@with_tree
def open_container(object_designator, arm, distance):
    object_type = object_designator.prop_value('type')
    if object_type in ["container", "drawer"]:
        motion_type = "opening-prismatic"
    elif object_type in ["fridge"]:
        motion_type = "opening-rotational"
    else:
        raise NotImplementedError()
    joint, handle = get_container_joint_and_handle(object_designator)
    arm = "left" if arm == Arms.LEFT else "right"
    environment = object_designator.prop_value('part-of')
    print("Plan distance: " + str(distance))
    ProcessModule.perform(MotionDesignator(
        [('type', motion_type), ('joint', joint),
         ('handle', handle), ('arm', arm), ('distance', distance),
         ('part-of', environment)]))

@with_tree
def close_container(object_designator, arm):
    object_type = object_designator.prop_value('type')
    if object_type in ["container", "drawer"]:
        motion_type = "closing-prismatic"
    elif object_type in ["fridge"]:
        motion_type = "closing-rotational"
    else:
        raise NotImplementedError()
    joint, handle = get_container_joint_and_handle(object_designator)
    arm = "left" if arm == Arms.LEFT else "right"
    environment = object_designator.prop_value('part-of')
    ProcessModule.perform(MotionDesignator(
        [('type', motion_type), ('joint', joint),
         ('handle', handle), ('arm', arm), ('part-of', environment)]))
