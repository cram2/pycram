from threading import Lock

import numpy as np

from ..datastructures.world import World
from ..datastructures.enums import ExecutionType, ObjectType
from ..designator import ObjectDesignatorDescription
from ..designators.motion_designator import MoveMotion, DetectingMotion, MoveTCPMotion, MoveArmJointsMotion, \
    MoveJointsMotion, MoveGripperMotion, OpeningMotion, ClosingMotion
from ..local_transformer import LocalTransformer
from ..process_module import ProcessModuleManager, ProcessModule
from .default_process_modules import DefaultOpen, DefaultClose, DefaultMoveGripper, DefaultMoveJoints, DefaultMoveTCP, \
    DefaultNavigation, DefaultMoveHead, DefaultDetecting, DefaultMoveArmJoints, DefaultWorldStateDetecting
from ..robot_description import RobotDescription
from ..ros.logging import logdebug
from ..external_interfaces import giskard
from ..world_concepts.world_object import Object
from ..external_interfaces.robokudo import send_query


class TiagoNavigationReal(ProcessModule):
    def _execute(self, designator: MoveMotion):
        logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, RobotDescription.current_robot_description.base_link, "map")

class TiagoMoveHeadReal(ProcessModule):
    def _execute(self, designator: MoveMotion):
        target = designator.target
        robot = World.robot

        local_transformer = LocalTransformer()

        pan_link = RobotDescription.current_robot_description.kinematic_chains["neck"].links[0]
        tilt_link = RobotDescription.current_robot_description.kinematic_chains["neck"].links[1]

        pan_joint = RobotDescription.current_robot_description.kinematic_chains["neck"].joints[0]
        tilt_joint = RobotDescription.current_robot_description.kinematic_chains["neck"].joints[1]
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame(pan_link))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame(tilt_link))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_position(pan_joint)
        current_tilt = robot.get_joint_position(tilt_joint)

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({pan_link: new_pan + current_pan,
                                    tilt_link: new_tilt + current_tilt})

class TiagoDetectingReal(ProcessModule):
    def _execute(self, designator: DetectingMotion):
        query_result = send_query(ObjectDesignatorDescription(types=[designator.object_type]))
        # print(query_result)
        obj_pose = query_result["ClusterPoseBBAnnotator"]

        lt = LocalTransformer()
        obj_pose = lt.transform_pose(obj_pose, World.robot.get_link_tf_frame("torso_lift_link"))
        obj_pose.orientation = [0, 0, 0, 1]
        obj_pose.position.x += 0.05

        world_obj = World.current_world.get_object_by_type(designator.object_type)
        if world_obj:
            world_obj[0].set_pose(obj_pose)
            return world_obj[0]
        elif designator.object_type == ObjectType.JEROEN_CUP:
            cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=obj_pose)
            return cup
        elif designator.object_type == ObjectType.BOWL:
            bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=obj_pose)
            return bowl

        return world_obj[0]

class TiagoMoveTCPReal(ProcessModule):
    def _execute(self, designator: MoveTCPMotion):
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")

        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, RobotDescription.current_robot_description.get_arm_chain(
            designator.arm).get_tool_frame(),
                                       "torso_lift_link")

class TiagoMoveArmJointsReal(ProcessModule):
    def _execute(self, designator: MoveArmJointsMotion):
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)

class TiagoMoveJointsReal(ProcessModule):
    def _execute(self, designator: MoveJointsMotion):
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)

class TiagoMoveGripperReal(ProcessModule):
    def _execute(self, designator: MoveGripperMotion):
        pass
        # TODO implement for the new tiago gripper

class TiagoOpenReal(ProcessModule):
    def _execute(self, designator: OpeningMotion):
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)

class TiagoCloseReal(ProcessModule):
    def _execute(self, designator: ClosingMotion):
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)



class TiagoManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("tiago_dual")
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
            return DefaultNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TiagoNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TiagoMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TiagoDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TiagoMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TiagoMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if (ProcessModuleManager.execution_type == ExecutionType.SIMULATED or
                ProcessModuleManager.execution_type == ExecutionType.REAL):
            return DefaultWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TiagoMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TiagoMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TiagoOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultClose(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TiagoCloseReal(self._close_lock)
