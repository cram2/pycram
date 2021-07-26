from pycram.motion_designator import *
from pycram.bullet_world import BulletWorld
from pycram.robot_description import InitializedRobotDescription as robot_description
import rospy

"""
The grounding functions for the Motion designator descriptions in pycram/motion_designator.py
They all inferre missing properties and return a dictionaty with the properties as
value.
"""


def ground_move(self):
    if not self.orientation:
        # get_orientation returns tuple, the conversion is because of the type
        # check in _check_properties
        self.orientation = list(BulletWorld.robot.get_orientation())
    self._check_properties("[Motion Designator] Moving")
    return self.__dict__

def ground_pick_up(self):
    if not self.arm:
        self.arm = 'left'
    self.gripper = robot_description.i.get_tool_frame(self.arm)
    self._check_properties("[Motion Designator] Pick-Up")
    return self.__dict__

def ground_place(self):
    if not self.arm:
        self.arm = 'left'
    self.gripper = robot_description.i.get_tool_frame(self.arm)
    self._check_properties("[Motion Designator] Place")
    return self.__dict__

def ground_accessing(self):
    if not self.arm:
        self.arm = 'left'
    self.gripper = robot_description.i.get_tool_frame(self.arm)
    self._check_properties("[Motion Designator] Accessing")
    return self.__dict__

def ground_move_tcp(self):
    if not self.arm:
        self.arm = 'left'
    self._check_properties("[Motion Designator] Move-TCP")
    return self.__dict__

def ground_looking(self):
    if self.target and self.object:
        rospy.logwarn(f"[Looking Designator Resolution] Target and Object parameter provided. Only Object will be used.")
        return {'cmd': self.cmd,
                'target': BulletWorld.current_bullet_world.get_objects_by_name(self.object)[0].get_pose}
    if self.object:
        return {'cmd': self.cmd,
                'target': BulletWorld.current_bullet_world.get_objects_by_name(self.object)[0].get_pose}
    if self.target:
        return self.make_dictionary(["cmd", "target"])
    if not self.target and not self.object:
        self._check_properties("[Motion Designator] Looking")

def ground_move_gripper(self):
    self._check_properties("[Motion Designator] Move-gripper")
    return self.__dict__

def ground_detect(self):
    if not self.cam_frame:
        self.cam_frame = robot_description.i.get_camera_frame()
    if not self.front_facing_axis:
        self.front_facing_axis = robot_description.i.front_facing_axis
    self._check_properties("[Motion Designator] Detecting")
    return self.__dict__

def ground_move_arm(self):
    if self.left_arm_config or self.right_arm_config:
        self.left_arm_poses = self.left_arm_config
        self.right_arm_poses = self.right_arm_config
        return self.__dict__
    if self.right_arm_poses or self.left_arm_poses:
        return self.__dict__
    else:
        self._check_properties("[Motion Designator] Move-arm-joints")

def ground_world_state_detecting(self):
    self._check_properties("[Motion Designator] World-state-detecting")
    return self.__dict__

MoveMotionDescription.ground = ground_move
PickUpMotionDescription.ground = ground_pick_up
PlaceMotionDescription.ground = ground_place
AccessingMotionDescription.ground = ground_accessing
MoveTCPMotionDescription.ground = ground_move_tcp
LookingMotionDescription.ground = ground_looking
MoveGripperMotionDescription.ground = ground_move_gripper
DetectingMotionDescription.ground = ground_detect
MoveArmJointsMotionDescription.ground = ground_move_arm
WorldStateDetectingMotionDescription.ground = ground_world_state_detecting

def call_ground(desig):
    return [desig._description.ground()]

MotionDesignator.resolvers.append(call_ground)
