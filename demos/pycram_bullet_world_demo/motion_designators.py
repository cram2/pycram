from pycram.motionDesignator import *
from pycram.designator import ResolutionError
from pycram.bullet_world import BulletWorld
from pycram.robot_description import InitializedRobotDescription as robot_description
import rospy
def ground_move(self):
    if not self.orientation:
        self.orientation = BulletWorld.robot.get_orientation()
    missing, wrong_type = self._check_properties()
    if missing != [] or wrong_type != {}:
        raise ResolutionError(missing, wrong_type, "[Motion Designator] Moving")
    return self.__dict__

def ground_pick_up(self):
    if not self.arm:
        self.arm = 'left'
    missing = self._check_properties()
    if missing != []:
        raise ResolutionError(missing, "[Motion Designator] Pick-Up")
    return self.__dict__

def ground_place(self):
    if not self.arm:
        self.arm = 'left'
    missing = self._check_properties()
    if missing != []:
        raise ResolutionError(missing, "[Motion Designator] Place")
    return self.__dict__

def ground_accessing(self):
    if not self.arm:
        self.arm = 'left'
    missing = self._check_properties()
    if missing != []:
        raise ResolutionError(missing, "[Motion Designator] Accessing")
    return self.__dict__

def ground_move_tcp(self):
    if not self.arm:
        self.arm = 'left'
    missing = self._check_properties()
    if missing != []:
        raise ResolutionError(missing, "[Motion Designator] Move-TCP")
    return sefl.__dict__

def ground_looking(self):
    if self.target and self.object:
        rospy.logwarn(f"[Looking Designator Resolution] Target and Object parameter provided. Only Object will be further used.")
        return {'cmd': self.cmd,
                'target': BulletWorld.current_bullet_world.get_objects_by_name(self.object).get_pose}
    if self.object:
        return {'cmd': self.cmd,
                'target': BulletWorld.current_bullet_world.get_objects_by_name(self.object).get_pose}
    if self.target:
        return self.make_dictionary(["cmd", "target"])
    if not self.target and not self.object:
        raise Resolution(['target', 'object'], "[Motion Designator] Looking")

def ground_move_gripper(self):
    missing = self._check_properties()
    if missing != []:
        raise ResolutionError(missing, "[Motion Designator] Move-gripper")
    return self.__dict__

def ground_detect(self):
    if not self.cam_frame:
        self.cam_frame = robot_description.i.get_camera_frame()
    if not self.front_facing_axis:
        self.front_facing_axis = robot.i.front_facing_axis
    missing = self._check_properties()
    if missing != []:
        raise ResolutionError(missing, "[Motion Designator] Detecting")
    return self.__dict__

def ground_move_arm(self):
    missing = self._check_properties()
    if missing != []:
        raise ResolutionError(missing, "[Motion Designator] Move-arm-joints")
    return self.__dict__

def ground_world_state_detecting(self):
    missing = self._check_properties()
    if missing != []:
        raise ResolutionError(missing, "[Motion Designator] World-state-detecting")
    return self.__dict__

MoveMotionDescription.ground = ground_move
PickUpMotionDescription.ground = ground_pick_up
PlaceMotionDescription.ground = ground_place
AccessingMotinDescription.ground = ground_accessing
MoveTCPMotinDescription.ground = ground_move_tcp
LookingMotionDescription.ground = ground_looking
MoveGripperMotionDescription.ground = ground_move_gripper
DetectingMotionDescription.ground = ground_detect
MoveArmJointsMotionDescription.ground = ground_move_arm
WorldStateDetectingMotionDescription.ground = ground_world_state_detecting

def call_ground(desig):
    return [desig._description.ground()]

MotionDesignator.resolvers.append(call_ground)

def pr2_motion_designators(desig):
    """
    This method defines the referencing of all available motion designator.
    If a possible solution is found it will be appended to the list of solutions.
    :param desig: The designator to be referenced
    :return: A list with possible solution.
    """
    solutions = []

    # Type: moving
    if desig.check_constraints([('type', 'moving'), 'target']):
        if desig.check_constraints(['orientation']):
            solutions.append(desig.make_dictionary([('cmd', 'navigate'), 'target', 'orientation']))
        solutions.append(desig.make_dictionary([('cmd', 'navigate'), 'target', ('orientation', BulletWorld.robot.get_orientation())]))

    # Type: pick-up
    if desig.check_constraints([('type', 'pick-up'), 'object']):
        if desig.check_constraints([('arm', 'right')]):
            solutions.append(desig.make_dictionary([('cmd', 'pick'), 'object', ('gripper', robot_description.i.get_tool_frame('right'))]))
        solutions.append(desig.make_dictionary([('cmd', 'pick'), 'object', ('gripper', robot_description.i.get_tool_frame('left'))]))

    # Type: place
    if desig.check_constraints([('type', 'place'), 'target']):
        if desig.check_constraints(['object']):
            if desig.check_constraints([('arm', 'right')]):
                solutions.append(desig.make_dictionary([('cmd', 'place'), 'target', 'object', ('gripper', robot_description.i.get_tool_frame('right'))]))
            solutions.append(desig.make_dictionary([('cmd', 'place'), 'target', 'object', ('gripper', robot_description.i.get_tool_frame('left'))]))

    # Type: accessing
    if desig.check_constraints([('type', 'accessing'), 'drawer-joint', 'drawer-handle', 'part-of']):
        if desig.check_constraints([('arm', 'right')]):
            if desig.check_constraints('distance'):
                solutions.append(desig.make_dictionary([('cmd', 'access'), 'drawer-joint', 'drawer-handle', ('gripper', robot_description.i.get_tool_frame('right')), 'distance', 'part-of']))
            solutions.append(desig.make_dictionary([('cmd', 'access'), 'drawer-joint', 'drawer-handle', ('gripper', robot_description.i.get_tool_frame('right')), ('distance', 0.3), 'part-of']))
        solutions.append(desig.make_dictionary([('cmd', 'access'), 'drawer-joint', 'drawer-handle', 'part-of', ('distance', 0.3), ('gripper', robot_description.i.get_tool_frame('left')), 'part-of']))

    # Type: move-tcp
    if desig.check_constraints([('type', 'move-tcp'), 'target']):
        if desig.check_constraints([('arm', 'right')]):
            solutions.append(desig.make_dictionary([('cmd', 'move-tcp'), 'target', ('gripper', robot_description.i.get_tool_frame('right'))]))
        solutions.append(desig.make_dictionary([('cmd', 'move-tcp'), 'target', ('gripper', robot_description.i.get_tool_frame('left'))]))

    # Type: park-arms
    if desig.check_constraints([('type', 'park-arms')]):
        solutions.append(desig.make_dictionary([('cmd', 'park')]))

    # Type: looking
    if desig.check_constraints([('type', 'looking')]):
        if desig.check_constraints(['target']):
            solutions.append(desig.make_dictionary([('cmd', 'looking'), 'target']))
        if desig.check_constraints(['object']):
            solutions.append(desig.make_dictionary([('cmd', 'looking'), ('target', BulletWorld.current_bullet_world.
                                                                         get_objects_by_name(desig.prop_value('object')).get_pose())]))

    # Type: opening-gripper
    if desig.check_constraints([('type', 'opening-gripper'), 'gripper']):
        solutions.append(desig.make_dictionary([('cmd', 'move-gripper'), ('motion', 'open'), 'gripper']))

    # Type: closing-gripper
    if desig.check_constraints([('type', 'closing-gripper'), 'gripper']):
        solutions.append(desig.make_dictionary([('cmd', 'move-gripper'), ('motion', 'close'), 'gripper']))

    # Type: detecting
    if desig.check_constraints([('type', 'detecting'), 'object']):
        solutions.append(desig.make_dictionary([('cmd', 'detecting'), ('cam_frame', robot_description.i.get_camera_frame()), ('front_facing_axis', [0, 0, 1]), 'object']))

    # Type: move-arm-joints
    if desig.check_constraints([('type', 'move-arm-joints')]):
        if desig.check_constraints(['left-arm', 'right-arm']):
            solutions.append(desig.make_dictionary([('cmd', 'move-joints'), ('left-poses', desig.prop_value('left-arm')), ('right-poses', desig.prop_value('right-arm'))]))
        if desig.check_constraints(['left-arm']):
            solutions.append(desig.make_dictionary([('cmd', 'move-joints'), ('left-poses', desig.prop_value('left-arm')), ('right-poses', None)]))
        if desig.check_constraints(['right-arm']):
            solutions.append(desig.make_dictionary([('cmd', 'move-joints'), ('right-poses', desig.prop_value('right-arm')), ('left-poses', None)]))

    # Type: world-state-detecting
    if desig.check_constraints([('type', 'world-state-detecting')]):
        solutions.append(desig.make_dictionary([('cmd', 'world-state-detecting'), 'object']))

    return solutions


#MotionDesignator.resolvers.append(pr2_motion_designators)
