from .default_process_modules import *
from ..datastructures.world import World
from pycram.robot_plans import *
from ..external_interfaces.robokudo import *
from ..robot_description import RobotDescription


class StretchMoveHead(ProcessModule):
    """
    Process module for the simulated Stretch that moves the head such that it looks at the given position
    """

    def _execute(self, designator: MoveMotion) -> Any:
        target = designator.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_pan"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        current_pan = robot.get_joint_position("joint_head_pan")

        robot.set_joint_position("joint_head_pan", new_pan + current_pan)

        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_tilt"))
        new_tilt = np.arctan2(-pose_in_tilt.position.y,
                              np.sqrt(pose_in_tilt.position.z ** 2 + pose_in_tilt.position.x ** 2)) * -1
        current_tilt = robot.get_joint_position("joint_head_tilt")

        robot.set_joint_position("joint_head_tilt", new_tilt + current_tilt)


###########################################################
########## Process Modules for the Real Stretch ###########
###########################################################


class StretchNavigationReal(ProcessModule):
    """
    Process module for the real Stretch that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, RobotDescription.current_robot_description.base_link, "map")


class StretchMoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)

        current_pan = robot.get_joint_position("joint_head_pan")
        current_tilt = robot.get_joint_position("joint_head_tilt")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({"head_pan_joint": new_pan + current_pan})

        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))
        new_tilt = np.arctan2(-pose_in_tilt.position.y,
                              np.sqrt(pose_in_tilt.position.z ** 2 + pose_in_tilt.position.x ** 2)) * -1
        current_tilt = robot.get_joint_position("joint_head_tilt")
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({"head_tilt_joint": new_tilt + current_tilt})


class StretchDetectingReal(ProcessModule):
    """
    Process Module for the real Stretch that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, designator: DetectingMotion) -> Any:
        query_result = query(ObjectDesignatorDescription(types=[designator.object_type]))
        # print(query_result)
        obj_pose = query_result["ClusterPoseBBAnnotator"]

        lt = LocalTransformer()
        obj_pose = lt.transform_pose(obj_pose, World.robot.get_link_tf_frame("torso_lift_link"))
        obj_pose.orientation = [0, 0, 0, 1]
        obj_pose.position.x += 0.05

        bullet_obj = World.current_world.get_objects_by_type(designator.object_type)
        if bullet_obj:
            bullet_obj[0].set_pose(obj_pose)
            return bullet_obj[0]
        elif designator.object_type == ObjectType.JEROEN_CUP:
            cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=obj_pose)
            return cup
        elif designator.object_type == ObjectType.BOWL:
            bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=obj_pose)
            return bowl

        return bullet_obj[0]


class StretchMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real Stretch, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        chain = RobotDescription.current_robot_description.get_arm_chain(designator.gripper).get_static_gripper_state(
            designator.motion)
        giskard.achieve_joint_goal(chain)


class StretchManager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "stretch_description"

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return StretchMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchMoveHeadReal(self._looking_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchMoveGripperReal(self._move_gripper_lock)

StretchManager()