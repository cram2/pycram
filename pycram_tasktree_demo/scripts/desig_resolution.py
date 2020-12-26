from pycram.pr2_description import Arms, Grasp
from pycram.designator import DesignatorError, ObjectDesignator
from pycram.action_designator import SetGripperActionDescription, PickUpDescription, PlaceDescription, \
    NavigateDescription, ParkArmsDescription, DetectActionDescription, LookAtActionDescription, \
    TransportObjectDescription, OpenActionDescription, CloseActionDescription
from plans import open_gripper, close_gripper, pick_up, place, navigate, park_arms, detect, look_at, transport, \
    open_container, close_container

def ground_set_gripper(self):
    if self.opening == 0:
        self.function = lambda : close_gripper(self.gripper)
    else:
        self.function = lambda : open_gripper(self.gripper)
    return super(SetGripperActionDescription, self).ground()

def ground_pick_up(self):
    if not self.object_designator:
        raise DesignatorError()
    if not self.arm:
        self.arm = Arms.LEFT
    if not self.grasp:
        self.grasp = Grasp.TOP
    self.gripper_opening = 0.9
    self.function = lambda: pick_up(self.arm, self.object_designator)
    return super(PickUpDescription, self).ground()

def ground_place(self):
    if not self.object_designator:
        raise DesignatorError()
    if not self.target_location:
        raise DesignatorError()
    if not self.arm:
        self.arm = Arms.LEFT
    self.function = lambda: place(self.arm, self.object_designator, self.target_location)
    return super(PlaceDescription, self).ground()

def ground_navigate(self : NavigateDescription):
    if self.target_orientation:
        self.function = lambda : navigate(self.target_position, self.target_orientation)
    else:
        self.function = lambda : navigate(self.target_position)
    return super(NavigateDescription, self).ground()

def ground_park_arms(self : ParkArmsDescription):
    self.function = lambda : park_arms(self.arm)
    return super(ParkArmsDescription, self).ground()

def ground_detect(self):
    self.function = lambda : detect(self.object_designator)
    return super(DetectActionDescription, self).ground()

def ground_look_at(self):
    if isinstance(self.target, list) or isinstance(self.target, tuple):
        self.function = lambda : look_at(self.target)
    elif isinstance(self.target, ObjectDesignator):
        object_name = self.target.prop_value('name')
        if object_name is 'iai_fridge':
            pos = [0.95, -0.9, 0.8]
        elif object_name is 'sink_area_left_upper_drawer':
            pos = [1.0, 0.7, 0.75]
        elif object_name is 'sink_area_left_middle_drawer':
            pos = [1.0, 0.925, 0.5]
        else:
            raise DesignatorError()
        self.function = lambda : look_at(pos)
    else:
        raise DesignatorError()
    return super(LookAtActionDescription, self).ground()

def ground_transport(self:TransportObjectDescription):
    self.function = lambda : transport(self.object_designator, self.arm, self.target_location)
    return super(TransportObjectDescription, self).ground()

def ground_open(self:OpenActionDescription):
    if not self.distance:
        if self.object_designator.prop_value('type') is 'fridge':
            self.distance = 1.0
        else:
            self.distance = 0.4
    self.function = lambda : open_container(self.object_designator, self.arm, self.distance)
    return super(OpenActionDescription, self).ground()

def ground_close(self:CloseActionDescription):
    self.function = lambda : close_container(self.object_designator, self.arm)
    return super(CloseActionDescription, self).ground()

SetGripperActionDescription.ground = ground_set_gripper
PickUpDescription.ground = ground_pick_up
PlaceDescription.ground = ground_place
NavigateDescription.ground = ground_navigate
ParkArmsDescription.ground = ground_park_arms
DetectActionDescription.ground = ground_detect
LookAtActionDescription.ground = ground_look_at
TransportObjectDescription.ground = ground_transport
OpenActionDescription.ground = ground_open
CloseActionDescription.ground = ground_close
