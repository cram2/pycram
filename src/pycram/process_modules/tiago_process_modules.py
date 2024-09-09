from .default_process_modules import *
from .default_process_modules import _move_arm_tcp
from ..datastructures.enums import Grasp


class TiagoMoveHead(DefaultMoveHead):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()

        # since its usually joint1, link1, joint2, link2, to be able to catch joint1, even though we only use the
        # kinematic chain from link1 to link2, we need to use the link0 as the first link, even though its technically
        # not in this kinematic chain
        pan_link = RobotDescription.current_robot_description.kinematic_chains["neck"].links[1]
        tilt_link = RobotDescription.current_robot_description.kinematic_chains["neck"].links[2]

        pan_joint = RobotDescription.current_robot_description.kinematic_chains["neck"].joints[0]
        tilt_joint = RobotDescription.current_robot_description.kinematic_chains["neck"].joints[1]
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame(pan_link)).position_as_list()
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame(tilt_link)).position_as_list()

        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])

        tilt_offset = RobotDescription.current_robot_description.get_offset(tilt_joint)
        tilt_offset_rotation = tilt_offset.rotation if tilt_offset else [0, 0, 0]
        rotation_tilt_offset = R.from_euler('xyz', tilt_offset_rotation).apply(pose_in_tilt)

        new_tilt = np.arctan2(rotation_tilt_offset[2],
                              np.sqrt(rotation_tilt_offset[0] ** 2 + rotation_tilt_offset[1] ** 2))

        current_pan = robot.get_joint_position(pan_joint)
        current_tilt = robot.get_joint_position(tilt_joint)

        robot.set_joint_position(pan_joint, new_pan + current_pan)
        robot.set_joint_position(tilt_joint, new_tilt + current_tilt)


class TiagoOpen(DefaultOpen):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        goal_pose.set_orientation([0, 0, 0, 1])
        grasp = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).end_effector.grasps[Grasp.FRONT]
        goal_pose.multiply_quaternions(grasp)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[1])


class TiagoClose(DefaultClose):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        goal_pose.set_orientation([0, 0, 0, 1])
        grasp = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).end_effector.grasps[Grasp.FRONT]
        goal_pose.multiply_quaternions(grasp)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[0])


class TiagoManager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "tiago_dual"

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return TiagoMoveHead(self._looking_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return TiagoOpen(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return TiagoClose(self._close_lock)