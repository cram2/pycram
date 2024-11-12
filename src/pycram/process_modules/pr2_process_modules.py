from typing_extensions import TYPE_CHECKING

import actionlib

from .default_process_modules import DefaultDetectingReal, DefaultDetecting
from .. import world_reasoning as btr
import numpy as np

from .. import world_reasoning as btr
from ..external_interfaces.move_base import query_pose_nav
from ..process_module import ProcessModule, ProcessModuleManager
from ..external_interfaces.ik import request_ik
from ..ros.logging import logdebug
from ..utils import _apply_ik
from ..local_transformer import LocalTransformer

from .. import world_reasoning as btr
from ..designators.motion_designator import MoveMotion, LookingMotion, \
    DetectingMotion, MoveTCPMotion, MoveArmJointsMotion, WorldStateDetectingMotion, MoveJointsMotion, \
    MoveGripperMotion, OpeningMotion, ClosingMotion
from ..robot_description import RobotDescription
from ..datastructures.world import World
from ..world_concepts.world_object import Object
from ..datastructures.pose import Pose
from ..datastructures.enums import JointType, ObjectType, Arms, ExecutionType, MovementType, GripperState
from ..external_interfaces import giskard
from ..external_interfaces.robokudo import *
from ..ros.logging import loginfo, logwarn, logdebug

if TYPE_CHECKING:
    from ..designators.object_designator import ObjectDesignatorDescription

try:
    from ..worlds.multiverse import Multiverse
except ImportError:
    Multiverse = type(None)

try:
    from control_msgs.msg import GripperCommandGoal, GripperCommandAction
except ImportError:
    logwarn("control_msgs import failed")

try:
    from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2
except ImportError:
    logdebug("Pr2GripperCommandGoal not found")


class Pr2Navigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        robot = World.robot
        robot.set_pose(desig.target)


class Pr2MoveHead(ProcessModule):
    """
        This process module moves the head to look at a specific point in the world coordinate frame.
        This point can either be a position or an object.
        """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_position("head_pan_joint")
        current_tilt = robot.get_joint_position("head_tilt_joint")

        robot.set_joint_position("head_pan_joint", new_pan + current_pan)
        robot.set_joint_position("head_tilt_joint", new_tilt + current_tilt)


class Pr2MoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion):
        robot = World.robot
        motion = desig.motion
        for joint, state in RobotDescription.current_robot_description.get_arm_chain(
                desig.gripper).get_static_gripper_state(motion).items():
            robot.set_joint_position(joint, state)


class Pr2MoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion):
        target = desig.target
        robot = World.robot

        _move_arm_tcp(target, robot, desig.arm)


class Pr2MoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion):

        robot = World.robot
        if desig.right_arm_poses:
            robot.set_multiple_joint_positions(desig.right_arm_poses)
        if desig.left_arm_poses:
            robot.set_multiple_joint_positions(desig.left_arm_poses)


class PR2MoveJoints(ProcessModule):
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """

    def _execute(self, desig: MoveJointsMotion):
        robot = World.robot
        robot.set_multiple_joint_positions(dict(zip(desig.names, desig.positions)))


class Pr2WorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.obj_type == obj_type, World.current_world.objects))[0]


class Pr2Open(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[1] - 0.05)


class Pr2Close(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: Arms) -> None:
    gripper = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

    joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv)


###########################################################
########## Process Modules for the Real PR2 ###############
###########################################################


class Pr2NavigationReal(ProcessModule):
    """
    Process module for the real PR2 that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        logdebug(f"Sending goal to movebase to Move the robot")
        query_pose_nav(designator.target)


class Pr2MoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, np.sqrt(pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2)) * -1

        current_pan = robot.get_joint_position("head_pan_joint")
        current_tilt = robot.get_joint_position("head_tilt_joint")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({"head_pan_joint": new_pan + current_pan,
                                    "head_tilt_joint": new_tilt + current_tilt})


class Pr2MoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real PR2 while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")
        tip_link = RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame()
        root_link = "map"

        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm.name.lower())

        if designator.movement_type == MovementType.STRAIGHT_TRANSLATION:
            giskard.achieve_straight_translation_goal(pose_in_map.position_as_list(), tip_link, root_link)
        elif designator.movement_type == MovementType.STRAIGHT_CARTESIAN:
            giskard.achieve_straight_cartesian_goal(pose_in_map, tip_link, root_link)
        elif designator.movement_type == MovementType.TRANSLATION:
            giskard.achieve_translation_goal(pose_in_map.position_as_list(), tip_link, root_link)
        elif designator.movement_type == MovementType.CARTESIAN:
            giskard.achieve_cartesian_goal(pose_in_map, tip_link, root_link,
                                           use_monitor=World.current_world.conf.use_giskard_monitor)


class Pr2MoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real PR2 to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class Pr2MoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class Pr2MoveGripperMultiverse(ProcessModule):
    """
    Opens or closes the gripper of the real PR2, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        def activate_callback():
            loginfo("Started gripper Movement")

        def done_callback(state, result):
            loginfo(f"Reached goal {designator.motion}: {result.reached_goal}")

        def feedback_callback(msg):
            pass

        goal = GripperCommandGoal()
        goal.command.position = 0.0 if designator.motion == GripperState.CLOSE else 0.4
        goal.command.max_effort = 50.0
        if designator.gripper == "right":
            controller_topic = "/real/pr2/right_gripper_controller/gripper_cmd"
        else:
            controller_topic = "/real/pr2/left_gripper_controller/gripper_cmd"
        client = actionlib.SimpleActionClient(controller_topic, GripperCommandAction)
        loginfo("Waiting for action server")
        client.wait_for_server()
        client.send_goal(goal, active_cb=activate_callback, done_cb=done_callback, feedback_cb=feedback_callback)
        wait = client.wait_for_result()


class Pr2MoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real PR2, gripper uses an action server for this instead of giskard 
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        def activate_callback():
            loginfo("Started gripper Movement")

        def done_callback(state, result):
            loginfo(f"Reached goal {designator.motion}")

        def feedback_callback(msg):
            pass

        goal = Pr2GripperCommandGoal()
        goal.command.position = 0.0 if designator.motion == GripperState.CLOSE else 0.1
        goal.command.max_effort = 50.0
        if designator.gripper == Arms.RIGHT:
            controller_topic = "r_gripper_controller/gripper_action"
        else:
            controller_topic = "l_gripper_controller/gripper_action"
        client = actionlib.SimpleActionClient(controller_topic, Pr2GripperCommandAction)
        loginfo("Waiting for action server")
        client.wait_for_server()
        client.send_goal(goal, active_cb=activate_callback, done_cb=done_callback, feedback_cb=feedback_callback)
        wait = client.wait_for_result()


class Pr2OpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class Pr2CloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class Pr2Manager(ProcessModuleManager):

    def __init__(self):
        super().__init__("pr2")
        self._navigate_lock = Lock()
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
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return Pr2Navigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return Pr2NavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return Pr2MoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return Pr2MoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED or not robokudo_found:
            if not robokudo_found:
                logwarn("Robokudo not found, using simulated detection")
            return DefaultDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return Pr2MoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return Pr2MoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return Pr2MoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return Pr2MoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if (ProcessModuleManager.execution_type == ExecutionType.SIMULATED or
                ProcessModuleManager.execution_type == ExecutionType.REAL):
            return Pr2WorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return PR2MoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return Pr2MoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return Pr2MoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            if (isinstance(World.current_world, Multiverse) and
                    World.current_world.conf.use_multiverse_process_modules):
                return Pr2MoveGripperMultiverse(self._move_gripper_lock)
            else:
                return Pr2MoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return Pr2Open(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return Pr2OpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return Pr2Close(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return Pr2CloseReal(self._close_lock)
