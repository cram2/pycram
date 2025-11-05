import logging
from .default_process_modules import *

from ..datastructures.enums import ExecutionType, ObjectType
from ..designator import ObjectDesignatorDescription
from ..robot_plans import MoveMotion, DetectingMotion, MoveTCPMotion
from ..process_module import ProcessModuleManager, ProcessModule
from .default_process_modules import DefaultMoveGripper, DefaultMoveTCP, \
    DefaultNavigation, DefaultMoveHead
from ..robot_description import RobotDescription
from ..external_interfaces import giskard
from ..external_interfaces.robokudo import send_query

logger = logging.getLogger(__name__)

class TiagoNavigationReal(ProcessModule):
    def _execute(self, designator: MoveMotion):
        logger.debug(f"Sending goal to giskard to Move the robot")
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


class TiagoManager(DefaultManager):

    def __init__(self):
        super().__init__()
        self.robot_name = "tiago_dual"

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

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TiagoMoveTCPReal(self._move_tcp_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveGripperReal(self._move_gripper_lock)

TiagoManager()