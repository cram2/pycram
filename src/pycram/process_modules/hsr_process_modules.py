from threading import Lock

from typing import Optional
from ..robot_descriptions import robot_description
from ..process_module import ProcessModule, ProcessModuleManager
from ..world import World
from ..pose import Pose, Point
from ..helper import _apply_ik
import pycram.world_reasoning as btr
import pybullet as p
import logging
import time


def calculate_and_apply_ik(robot, gripper: str, target_position: Point, max_iterations: Optional[int] = None):
    """
    Calculates the inverse kinematics for the given target pose and applies it to the robot.
    """
    inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), target_position,
                                       maxNumIterations=max_iterations)
    # TODO: Check if this is correct (getting the arm and using its joints), previously joints was not provided.
    arm = "right" if gripper == robot_description.get_tool_frame("right") else "left"
    joints = robot_description.chains[arm].joints
    _apply_ik(robot, inv, joints)


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arm of HSR and applies them to the
    in the World defined robot.
    :return: None
    """

    robot = World.robot
    if arm == "left":
        for joint, pose in robot_description.get_static_joint_chain("left", "park").items():
            robot.set_joint_position(joint, pose)


class HSRNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'navigate':
            robot = World.robot
            robot.set_pose(Pose(solution['target'], solution['orientation']))


class HSRPickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'pick':
            obj = solution['object']
            robot = World.robot
            target = obj.get_position()

            calculate_and_apply_ik(robot, solution['gripper'], target, 100)

            robot.attach(obj, solution['gripper'])
            time.sleep(0.5)


class HSRPlace(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'place':
            obj = solution['object']
            robot = World.robot
            calculate_and_apply_ik(robot, solution['gripper'], solution['target'], 100)
            robot.detach(obj)
            time.sleep(0.5)


class HSRAccessing(ProcessModule):
    """
    This process module responsible for opening drawers to access the objects inside. This works by firstly moving
    the end effector to the handle of the drawer. Next, the end effector is moved the respective distance to the back.
    This provides the illusion the robot would open the drawer by himself.
    Then the drawer will be opened by setting the joint pose of the drawer joint.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'access':
            kitchen = solution['part_of']
            robot = World.robot
            gripper = solution['gripper']
            drawer_handle = solution['drawer_handle']
            drawer_joint = solution['drawer_joint']
            dis = solution['distance']
            calculate_and_apply_ik(robot, gripper, kitchen.links[drawer_handle].position)
            time.sleep(0.2)
            han_pose = kitchen.links[drawer_handle].position
            new_p = Point(han_pose[0] - dis, han_pose[1], han_pose[2])
            calculate_and_apply_ik(robot, gripper, new_p)
            kitchen.set_joint_position(drawer_joint, 0.3)
            time.sleep(0.5)


class HSRParkArms(ProcessModule):
    """
    This process module is for moving the arms in a parking position.
    It is currently not used.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'park':
            _park_arms()


class HSRMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'looking':
            target = solutions['target']
            if target == 'forward' or target == 'down':
                robot = World.robot
                for joint, state in robot_description.get_static_joint_chain("neck", target).items():
                    robot.set_joint_position(joint, state)
            else:
                logging.error("There is no target position defined with the target %s.", target)


class HSRMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-gripper":
            robot = World.robot
            gripper = solution['gripper']
            motion = solution['motion']
            for joint, state in robot_description.get_static_gripper_chain(gripper, motion).items():
                robot.set_joint_position(joint, state)
            time.sleep(0.5)


class HSRDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "detecting":
            robot = World.robot
            object_type = solution['object_type']
            cam_frame_name = solution['cam_frame']
            front_facing_axis = solution['front_facing_axis']

            objects = World.current_world.get_objects_by_type(object_type)
            for obj in objects:
                if btr.visible(obj, robot.links[cam_frame_name].pose, front_facing_axis, 0.5):
                    return obj


class HSRMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-tcp":
            target = solution['target']
            gripper = solution['gripper']
            robot = World.robot
            calculate_and_apply_ik(robot, gripper, target)
            time.sleep(0.5)


class HSRMoveJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-arm-joints":
            robot = World.robot
            left_arm_poses = solution['left_arm_poses']

            if type(left_arm_poses) == dict:
                for joint, pose in left_arm_poses.items():
                    robot.set_joint_position(joint, pose)
            elif type(left_arm_poses) == str and left_arm_poses == "park":
                _park_arms("left")

            time.sleep(0.5)


class HSRWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "world-state-detecting":
            obj_type = solution['object_type']
            return list(filter(lambda obj: obj.obj_type == obj_type, World.current_world.objects))[0]


class HSRManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("hsrb")
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
            return HSRNavigation(self._navigate_lock)

    def pick_up(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRPickUp(self._pick_up_lock)

    def place(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRPlace(self._place_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRMoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRMoveTCP(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRMoveJoints(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRWorldStateDetecting(self._world_state_detecting_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRMoveGripper(self._move_gripper_lock)
