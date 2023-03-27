from pycram.designator import DesignatorError
from pycram.designators.object_designator import ObjectDesignator
from pycram.designators.action_designator import *
from .plans import open_gripper, close_gripper, pick_up, place, navigate, park_arms, detect, look_at, transport, \
    open_container, close_container, move_torso
from enum import Enum, auto

class Arms(Enum):
    LEFT = auto()
    RIGHT = auto()
    BOTH = auto()

class Grasp(Enum):
    TOP = auto()

def ground_set_gripper(self):
    if self.opening == 0:
        self.function = lambda : close_gripper(self.gripper)
    else:
        self.function = lambda : open_gripper(self.gripper)
    return super(SetGripperAction, self).ground()

def ground_pick_up(self):
    if not self.object_designator:
        raise DesignatorError()
    if not self.arm:
        self.arm = Arms.LEFT
    if not self.grasp:
        self.grasp = Grasp.TOP
    self.gripper_opening = 0.9
    self.function = lambda: pick_up(self.arm, self.object_designator)
    return super(PickUpAction, self).ground()

def ground_place(self):
    if not self.object_designator:
        raise DesignatorError()
    if not self.target_location:
        raise DesignatorError()
    if not self.arm:
        self.arm = Arms.LEFT
    self.function = lambda: place(self.arm, self.object_designator, self.target_location)
    return super(PlaceAction, self).ground()

def ground_navigate(self : NavigateDescription):
    if self.target_orientation:
        self.function = lambda : navigate(self.target_position, self.target_orientation)
    else:
        self.function = lambda : navigate(self.target_position)
    return super(NavigateAction, self).ground()

def ground_park_arms(self : ParkArmsDescription):
    self.function = lambda : park_arms(self.arm)
    return super(ParkArmsAction, self).ground()

def ground_detect(self):
    self.function = lambda : detect(self.object_designator)
    return super(DetectAction, self).ground()

def ground_look_at(self):
    if isinstance(self.target, list) or isinstance(self.target, tuple):
        self.function = lambda : look_at(self.target)
    elif isinstance(self.target, ObjectDesignator):
        object_name = self.target.prop_value('name')
        if object_name == 'iai_fridge':
            pos = [0.95, -0.9, 0.8]
        elif object_name == 'sink_area_left_upper_drawer':
            pos = [1.0, 0.7, 0.75]
        elif object_name == 'sink_area_left_middle_drawer':
            pos = [1.0, 0.925, 0.5]
        else:
            raise DesignatorError()
        self.function = lambda : look_at(pos)
    else:
        raise DesignatorError()
    return super(LookAtAction, self).ground()

def ground_transport(self:TransportObjectDescription):
    self.function = lambda : transport(self.object_designator, self.arm, self.target_location)
    return super(TransportAction, self).ground()

def ground_open(self:OpenAction):
    if not self.distance:
        if self.object_designator.prop_value('type') == 'fridge':
            self.distance = 1.0
        else:
            self.distance = 0.4
    self.function = lambda : open_container(self.object_designator, self.arm, self.distance)
    return super(OpenAction, self).ground()

def ground_close(self:CloseAction):
    self.function = lambda : close_container(self.object_designator, self.arm)
    return super(CloseAction, self).ground()

def ground_move_torso(self):
    self.function = lambda : move_torso(self.position)
    return super(MoveTorsoAction, self).ground()

def call_ground(desig):
    type_to_ground = {MoveTorsoAction: ground_move_torso,
                      SetGripperAction: ground_set_gripper,
                      PickUpAction: ground_pick_up,
                      PlaceAction: ground_place,
                      NavigateAction: ground_navigate,
                      ParkArmsAction: ground_park_arms,
                      DetectAction: ground_detect,
                      LookAtAction: ground_look_at,
                      TransportAction: ground_transport,
                      OpenAction: ground_open,
                      CloseAction: ground_close}
    ground_function = type_to_ground[type(desig.description)]
    return ground_function(desig.description)

ActionDesignator.resolver['grounding'] = call_ground
