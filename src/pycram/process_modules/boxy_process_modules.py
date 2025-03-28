from threading import Lock
import numpy as np

from .default_process_modules import DefaultManager
from .donbot_process_modules import _park_arms
from .. import world_reasoning as btr
from ..utils import _apply_ik
from ..designators.motion_designator import *
from ..datastructures.enums import JointType
from ..external_interfaces.ik import request_ik

from ..datastructures.world import World
from ..local_transformer import LocalTransformer
from ..process_module import ProcessModule, ProcessModuleManager
from ..robot_description import RobotDescription
from ..world_concepts.world_object import Object


class BoxyNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        robot = World.robot
        robot.set_pose(desig.target)


class BoxyOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[1])


class BoxyClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """
    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[0])


class BoxyParkArms(ProcessModule):
    """
    This process module is for moving the arms in a parking position.
    It is currently not used.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'park':
            _park_arms()


class BoxyMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()

        pose_in_shoulder = local_transformer.transform_pose(target, robot.get_link_tf_frame("neck_shoulder_link"))

        if pose_in_shoulder.position.x >= 0 and pose_in_shoulder.position.x >= abs(pose_in_shoulder.position.y):
            robot.set_multiple_joint_positions(RobotDescription.current_robot_description.get_static_joint_chain("neck", "front"))
        if pose_in_shoulder.position.y >= 0 and pose_in_shoulder.position.y >= abs(pose_in_shoulder.position.x):
            robot.set_multiple_joint_positions(RobotDescription.current_robot_description.get_static_joint_chain("neck", "neck_right"))
        if pose_in_shoulder.position.x <= 0 and abs(pose_in_shoulder.position.x) > abs(pose_in_shoulder.position.y):
            robot.set_multiple_joint_positions(RobotDescription.current_robot_description.get_static_joint_chain("neck", "back"))
        if pose_in_shoulder.position.y <= 0 and abs(pose_in_shoulder.position.y) > abs(pose_in_shoulder.position.x):
            robot.set_multiple_joint_positions(RobotDescription.current_robot_description.get_static_joint_chain("neck", "neck_left"))

        pose_in_shoulder = local_transformer.transform_pose(target, robot.get_link_tf_frame("neck_shoulder_link"))

        new_pan = np.arctan2(pose_in_shoulder.position.y, pose_in_shoulder.position.x)

        robot.set_joint_position("neck_shoulder_pan_joint",
                                 new_pan + robot.get_joint_position("neck_shoulder_pan_joint"))


class BoxyMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only move one gripper at a time.
    """

    def _execute(self, desig):
        robot = World.robot
        gripper = desig.gripper
        motion = desig.motion
        robot.set_multiple_joint_positions(RobotDescription.current_robot_description.kinematic_chains[gripper].get_static_gripper_state(motion))


class BoxyDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig):
        robot = World.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_link_name = RobotDescription.current_robot_description.get_camera_link()
        # should be [0, 0, 1]
        front_facing_axis = RobotDescription.current_robot_description.get_default_camera().front_facing_axis

        objects = World.current_world.get_object_by_type(object_type)
        for obj in objects:
            if btr.visible(obj, robot.get_link_pose(cam_link_name), front_facing_axis):
                return obj


class BoxyMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion):
        target = desig.target
        robot = World.robot

        _move_arm_tcp(target, robot, desig.arm)


class BoxyMoveArmJoints(ProcessModule):
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


class BoxyWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, World.current_world.objects))[0]


def _move_arm_tcp(target: PoseStamped, robot: Object, arm: Arms) -> None:
    gripper = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

    joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv)


class BoxyManager(DefaultManager):

    def __init__(self):
        super().__init__()
        self.robot_name = "boxy"


    def navigate(self):
        if ProcessModuleManager.execution_type ==  ExecutionType.SIMULATED:
            return BoxyNavigation(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type ==  ExecutionType.SIMULATED:
            return BoxyMoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type ==  ExecutionType.SIMULATED:
            return BoxyDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type ==  ExecutionType.SIMULATED:
            return BoxyMoveTCP(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type ==  ExecutionType.SIMULATED:
            return BoxyMoveArmJoints(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type ==  ExecutionType.SIMULATED:
            return BoxyWorldStateDetecting(self._world_state_detecting_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type ==  ExecutionType.SIMULATED:
            return BoxyMoveGripper(self._move_gripper_lock)
