import time
from threading import Lock

import numpy as np
import pybullet as p

import pycram.bullet_world_reasoning as btr
import pycram.helper as helper
from ..bullet_world import BulletWorld, Object
from ..designators.motion_designator import MoveArmJointsMotion, WorldStateDetectingMotion
from ..external_interfaces.ik import request_ik
from ..local_transformer import LocalTransformer
from ..pose import Pose
from ..process_module import ProcessModule, ProcessModuleManager
from ..robot_descriptions import robot_description
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arm of Donbot and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = BulletWorld.robot
    if arm == "left":
        for joint, pose in robot_description.get_static_joint_chain("left", "park").items():
            robot.set_joint_state(joint, pose)


class DonbotNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig):
        robot = BulletWorld.robot
        robot.set_pose(desig.target)

class DonbotPickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """

    def _execute(self, desig):
        object = desig.object_desig.bullet_world_object
        robot = BulletWorld.robot
        grasp = robot_description.grasps.get_orientation_for_grasp(desig.grasp)
        target = object.get_pose()
        target.orientation.x = grasp[0]
        target.orientation.y = grasp[1]
        target.orientation.z = grasp[2]
        target.orientation.w = grasp[3]

        _move_arm_tcp(target, robot, "left")
        tool_frame = robot_description.get_tool_frame("left")
        robot.attach(object, tool_frame)


class DonbotPlace(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """

    def _execute(self, desig):
        object = desig.object.bullet_world_object
        robot = BulletWorld.robot

        # Transformations such that the target position is the position of the object and not the tcp
        object_pose = object.get_pose()
        local_tf = LocalTransformer()
        tcp_to_object = local_tf.transform_pose(object_pose,
                                                robot.get_link_tf_frame(robot_description.get_tool_frame("left")))
        target_diff = desig.target.to_transform("target").inverse_times(tcp_to_object.to_transform("object")).to_pose()

        _move_arm_tcp(target_diff, robot, "left")
        robot.detach(object)



class DonbotMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig):
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()

        pose_in_shoulder = local_transformer.transform_pose(target, robot.get_link_tf_frame("ur5_shoulder_link"))

        if pose_in_shoulder.position.x >= 0 and pose_in_shoulder.position.x >= abs(pose_in_shoulder.position.y):
            robot.set_joint_states(robot_description.get_static_joint_chain("left", "front"))
        if pose_in_shoulder.position.y >= 0 and pose_in_shoulder.position.y >= abs(pose_in_shoulder.position.x):
            robot.set_joint_states(robot_description.get_static_joint_chain("left", "arm_right"))
        if pose_in_shoulder.position.x <= 0 and abs(pose_in_shoulder.position.x) > abs(pose_in_shoulder.position.y):
            robot.set_joint_states(robot_description.get_static_joint_chain("left", "back"))
        if pose_in_shoulder.position.y <= 0 and abs(pose_in_shoulder.position.y) > abs(pose_in_shoulder.position.x):
            robot.set_joint_states(robot_description.get_static_joint_chain("left", "arm_left"))

        pose_in_shoulder = local_transformer.transform_pose(target, robot.get_link_tf_frame("ur5_shoulder_link"))

        new_pan = np.arctan2(pose_in_shoulder.position.y, pose_in_shoulder.position.x)

        robot.set_joint_state("ur5_shoulder_pan_joint", new_pan + robot.get_joint_state("ur5_shoulder_pan_joint"))


class DonbotMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only move one gripper at a time.
    """

    def _execute(self, desig):
        robot = BulletWorld.robot
        gripper = desig.gripper
        motion = desig.motion
        robot.set_joint_states(robot_description.get_static_gripper_chain(gripper, motion))


class DonbotDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig):
        robot = BulletWorld.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = robot_description.get_camera_frame()
        # should be [0, 0, 1]
        front_facing_axis = robot_description.front_facing_axis

        objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)
        for obj in objects:
            if btr.visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                return obj


class DonbotMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig):
        target = desig.target
        robot = BulletWorld.robot

        _move_arm_tcp(target, robot, desig.arm)


class DonbotMoveJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion):
        robot = BulletWorld.robot
        if desig.left_arm_poses:
            robot.set_joint_states(desig.left_arm_poses)


class DonbotWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]

def _move_arm_tcp(target: Pose, robot: Object, arm: str) -> None:
    gripper = robot_description.get_tool_frame(arm)

    joints = robot_description.chains[arm].joints

    inv = request_ik(target, robot, joints, gripper)
    helper._apply_ik(robot, inv, joints)


class DonbotManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("iai_donbot")
        self._navigate_lock = Lock()
        self._pick_up_lock = Lock()
        self._place_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DonbotNavigation(self._navigate_lock)

    def pick_up(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DonbotPickUp(self._pick_up_lock)

    def place(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DonbotPlace(self._place_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DonbotMoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DonbotDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DonbotMoveTCP(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DonbotMoveJoints(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DonbotWorldStateDetecting(self._world_state_detecting_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DonbotMoveGripper(self._move_gripper_lock)
