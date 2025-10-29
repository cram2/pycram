from scipy.spatial.transform import Rotation as R
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.pr2 import PR2, AbstractRobot
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import TYPE_CHECKING

from ..datastructures.dataclasses import Colors
from ..datastructures.enums import ExecutionType
from ..external_interfaces import giskard
from ..external_interfaces.robokudo import query_all_objects, query_object, query_human, query_specific_region, \
    query_human_attributes, query_waving_human
from ..failures import NavigationGoalNotReachedError
from ..process_module import ProcessModule, ManagerBase
from ..robot_plans import *
from ..ros import get_time
from ..ros import logdebug, loginfo
from ..tf_transformations import euler_from_quaternion
from ..world_reasoning import visible, link_pose_for_joint_config

if TYPE_CHECKING:
    from ..designators.object_designator import ObjectDesignatorDescription


class DefaultNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):

        connection = desig.world.get_connection(desig.world.root, desig.robot_view.root)
        connection.origin = desig.target.to_spatial_type()
        desig.world.notify_state_change()


class DefaultMoveHead(ProcessModule):
    """
    Moves the robot's head to look at a specified target point in the world coordinate frame.
    The target can be either a position or an object.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        neck = ViewManager.get_neck_view(desig.robot_view)

        pan_link = neck.yaw_body
        tilt_link = neck.pitch_body

        pan_joint = pan_link.parent_connection
        tilt_joint = tilt_link.parent_connection

        pose_in_map = desig.world.transform(target.to_spatial_type(), desig.world.root)

        pose_in_pan = \
        desig.world.transform(pose_in_map, pan_link).to_np()[:3, 3]
        pose_in_tilt = \
        desig.world.transform(pose_in_map, tilt_link).to_np()[:3, 3]

        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])

        # tilt_offset = RobotDescription.current_robot_description.get_offset(tilt_joint)
        # if tilt_offset:
        #     tilt_offset_rotation = tilt_offset.orientation
        #     quaternion_list = [tilt_offset_rotation.x, tilt_offset_rotation.y, tilt_offset_rotation.z,
        #                        tilt_offset_rotation.w]
        # else:
        quaternion_list = [0, 0, 0, 1]

        tilt_offset_rotation = euler_from_quaternion(quaternion_list, axes='sxyz')
        adjusted_pose_in_tilt = R.from_euler('xyz', tilt_offset_rotation).apply(pose_in_tilt)

        new_tilt = -np.arctan2(adjusted_pose_in_tilt[2],
                               np.sqrt(adjusted_pose_in_tilt[0] ** 2 + adjusted_pose_in_tilt[1] ** 2))

        if desig.robot_view.name in {"iCub", "tiago_dual"}:
            new_tilt = -new_tilt

        current_pan = pan_joint.position
        current_tilt = tilt_joint.position
        pan_joint.position = new_pan + current_pan
        tilt_joint.position = new_tilt + current_tilt
        desig.world.notify_state_change()


class DefaultMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only move one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion):
        gripper_state = JointStateManager().get_gripper_state(desig.gripper, desig.motion, desig.robot_view)
        gripper_state.apply_to_world(desig.world)
        desig.world.notify_state_change()\

class DefaultDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    :return: A list of perceived objects.
    """

    def _execute(self, designator: DetectingMotion):
        robot = World.robot
        cam_link_name = RobotDescription.current_robot_description.get_camera_link()
        camera_description = RobotDescription.current_robot_description.cameras[
            list(RobotDescription.current_robot_description.cameras.keys())[0]]
        front_facing_axis = camera_description.front_facing_axis
        query_result = []
        world_objects = []

        if designator.technique == DetectionTechnique.TYPES:
            try:
                object_types = designator.object_designator_description.obj_type
            except AttributeError:
                raise AttributeError("The object designator does not contain a type attribute")

            list1 = World.current_world.get_object_by_type(object_types)
            world_objects = world_objects + list1
        elif designator.technique == DetectionTechnique.ALL:
            world_objects = World.current_world.get_scene_objects()
        elif designator.technique == DetectionTechnique.HUMAN:
            raise NotImplementedError("Detection by human is not yet implemented in simulation")
        elif designator.technique == DetectionTechnique.REGION:
            raise NotImplementedError("Detection by region is not yet implemented in simulation")
        elif designator.technique == DetectionTechnique.HUMAN_ATTRIBUTES:
            raise NotImplementedError("Detection by human attributes is not yet implemented in simulation")
        elif designator.technique == DetectionTechnique.HUMAN_WAVING:
            raise NotImplementedError("Detection by waving human is not yet implemented in simulation")
        for obj in world_objects:
            if visible(obj, robot.get_link_pose(cam_link_name), front_facing_axis):
                query_result.append(obj)
        if query_result is None:
            raise PerceptionObjectNotFound(
                f"Could not find an object with the type {object_types} in the FOV of the robot")
        else:
            object_dict = []

            for obj in query_result:
                object_dict.append(obj)

            return object_dict


class DefaultMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion):
        arm = ViewManager.get_arm_view(desig.arm, desig.robot_view)

        target = desig.target.to_spatial_type()

        inv = desig.world.compute_inverse_kinematics(desig.world.root, arm.manipulator.tool_frame, target, max_iterations=5000)

        for joint, state in inv.items():
            desig.world.state[joint.name].position = state
        desig.world.notify_state_change()


class DefaultMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: 'MoveArmJointsMotion'):

        if desig.right_arm_poses:
            for joint, position in desig.right_arm_poses.items():
                dof = desig.world.get_degree_of_freedom_by_name(joint)
                desig.world.state[dof.name].position = position

        if desig.left_arm_poses:
            for joint, position in desig.right_arm_poses.items():
                dof = desig.world.get_degree_of_freedom_by_name(joint)
                desig.world.state[dof.name].position = position
        desig.world.notify_state_change()


class DefaultMoveJoints(ProcessModule):
    def _execute(self, desig: MoveJointsMotion):
        for joint, position in zip(desig.names, desig.positions):
            dof = desig.world.get_degree_of_freedom_by_name(joint)
            desig.world.state[dof.name].position = position
        desig.world.notify_state_change()


class DefaultOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):

        # compute the chain of connections only works top down,
        handle_to_root_connections = list(reversed(desig.world.compute_chain_of_connections(desig.world.root, desig.object_part)))
        # Search for the first connection that is not a FixedConnection,
        container_connection = list(filter(lambda c: not isinstance(c, FixedConnection), handle_to_root_connections))[0]

        lower_limit = container_connection.dof.lower_limits.position
        upper_limit = container_connection.dof.upper_limits.position

        goal_pose = link_pose_for_joint_config(desig.object_part, {
            container_connection.dof.name.name: max(lower_limit, upper_limit - 0.05)})

        _move_arm_tcp(goal_pose, desig.robot_view, desig.arm, desig.world)

        container_connection.position = upper_limit


class DefaultClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion):
        # compute the chain of connections only works top down,
        handle_to_root_connections = list(reversed(desig.world.compute_chain_of_connections(desig.world.root, desig.object_part)))
        # Search for the first connection that is not a FixedConnection,
        container_connection = list(filter(lambda c: not isinstance(c, FixedConnection), handle_to_root_connections))[0]

        lower_limit = container_connection.dof.lower_limits.position
        upper_limit = container_connection.dof.upper_limits.position

        goal_pose = link_pose_for_joint_config(desig.object_part, {
            container_connection.dof.name.name: min(lower_limit, upper_limit - 0.05)})

        _move_arm_tcp(goal_pose, desig.robot_view, desig.arm, desig.world)

        container_connection.position = lower_limit


class DefaultMoveTCPWaypoints(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm along a list of waypoints.
    """

    def _execute(self, desig: MoveTCPWaypointsMotion):
        waypoints = desig.waypoints
        for waypoint in waypoints:
            _move_arm_tcp(waypoint, desig.robot_view, desig.arm, desig.world)


def _move_arm_tcp(target: PoseStamped, robot: AbstractRobot, arm: Arms, world: World) -> None:
    """
    Calls the ik solver to calculate the inverse kinematics of the arm and then sets the joint states accordingly.

    :param target: Target pose to which the end-effector should move.
    :param robot: Robot object representing the robot.
    :param arm: Which arm to move
    """
    tip = ViewManager().get_end_effector_view(arm, robot).tool_frame
    inv = world.compute_inverse_kinematics(world.root, tip, target.to_spatial_type())

    for joint, state in inv.items():
        world.state[joint.name].position = state
    world.notify_state_change()


###########################################################
########## Process Modules for the Real     ###############
###########################################################

class DefaultDetectingReal(ProcessModule):
    def _execute(self, designator: DetectingMotion) -> List[Body]:
        """
            Perform a query based on the detection technique and state defined in the designator.

            :return: A list of perceived objects.
            """
        print(designator.technique)
        object_designator_description = designator.object_designator_description
        query_methods = {
            DetectionTechnique.TYPES: lambda: query_object(object_designator_description),
            DetectionTechnique.HUMAN: lambda: query_human(),
            DetectionTechnique.HUMAN_ATTRIBUTES: query_human_attributes,
            DetectionTechnique.HUMAN_WAVING: query_waving_human,
            DetectionTechnique.REGION: lambda: query_specific_region(designator.region)
        }  # Fetch the appropriate query function
        query_func = query_methods.get(designator.technique, query_all_objects)
        query_result = query_func() if callable(query_func) else query_func
        # Handle the case where no result is found
        if query_result is None:
            raise PerceptionObjectNotFound(
                f"Could not find an object in the FOV of the robot")
        else:
            perceived_objects = []
            for i in range(0, len(query_result.res)):
                try:
                    obj_pose = PoseStamped.from_ros_message(query_result.res[i].pose[0])
                except IndexError:
                    obj_pose = PoseStamped.from_ros_message(query_result.res[i].pose)
                    pass
                obj_pose.frame_id = World.robot.get_link_tf_frame(obj_pose.frame_id)
                obj_pose_T_m = designator.world.transform(obj_pose, designator.world.root)

                obj_type = query_result.res[i].type
                obj_size = None
                try:
                    obj_size = query_result.res[i].shape_size[0].dimensions
                except IndexError:
                    pass

                obj_color = None
                try:
                    obj_color = query_result.res[i].color[0]
                except IndexError:
                    pass

                if obj_size is None:
                    hsize = [0.2, 0.2, 0.2]

                else:
                    hsize = [obj_size.x / 2, obj_size.y / 2, obj_size.z / 2]

                # Check if the object type is a subclass of the classes in the objects module (pycrap)
                type_concept = parse_furniture(obj_type)
                if type_concept is None:
                    loginfo(f"No class name contains the string '{obj_type}'")
                    type_concept = PhysicalObject

                obj_name = obj_type + "" + str(get_time())

                # gen_obj_desc = GenericObjectDescription(obj_name, [0, 0, 0], hsize)
                gen_obj = Body(name=PrefixedName(obj_name))
                designator.world.add_object(gen_obj)

                if obj_color is not None:
                    color = Colors.from_string(obj_color)
                else:
                    color = Colors.PINK

                # generic_obj = Object(name=obj_name, concept=type_concept, path=None, description=gen_obj_desc,
                #                      color=color)

                designator.world.get_connection(designator.world.root, gen_obj).origin = obj_pose

                perceived_objects.append(gen_obj)

            object_dict = []

            for obj in perceived_objects:
                object_dict.append(ObjectDesignatorDescription.Object(obj.name, obj.obj_type,
                                                                      obj))

            return object_dict


class DefaultNavigationReal(ProcessModule):
    """
    Process module for the real robot that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion):
        # logdebug(f"Sending goal to movebase to Move the robot")
        # query_pose_nav(designator.target)
        logdebug(f"Sending goal to giskard to Move the robot")
        # giskard.avoid_all_collisions()
        giskard.allow_self_collision()
        giskard.achieve_cartesian_goal(designator.target,
                                       RobotDescription.current_robot_description.base_link,
                                       "map")

        if not World.current_world.robot.pose.almost_equal(designator.target, 0.05, 3):
            raise NavigationGoalNotReachedError(World.current_world.robot.pose, designator.target)


class DefaultMoveHeadReal(ProcessModule):
    """
    Process module for controlling the real robot's head to look at a specified position.
    Uses the same calculations as the simulated version to orient the head.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = desig.world.get_views_by_type(AbstractRobot)

        neck = RobotDescription.current_robot_description.get_neck()
        pan_link = neck["yaw"][0]
        tilt_link = neck["pitch"][0]

        pan_joint = neck["yaw"][1]
        tilt_joint = neck["pitch"][1]

        pose_in_map = desig.world.transform_pose(target.to_spatial_type(), desig.world.root)

        pose_in_pan = \
            desig.world.transform(pose_in_map.to_spatial_type(), desig.world.get_body_by_name(pan_link)).to_np()[:3, 3]
        pose_in_tilt = \
            desig.world.transform(pose_in_map.to_spatial_type(), desig.world.get_body_by_name(tilt_link)).to_np()[:3, 3]

        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])

        tilt_offset = RobotDescription.current_robot_description.get_offset(tilt_joint)
        if tilt_offset:
            tilt_offset_rotation = tilt_offset.pose.orientation
            quaternion_list = [tilt_offset_rotation.x, tilt_offset_rotation.y, tilt_offset_rotation.z,
                               tilt_offset_rotation.w]
        else:
            quaternion_list = [0, 0, 0, 1]

        tilt_offset_rotation = euler_from_quaternion(quaternion_list, axes='sxyz')
        adjusted_pose_in_tilt = R.from_euler('xyz', tilt_offset_rotation).apply(pose_in_tilt)

        new_tilt = -np.arctan2(adjusted_pose_in_tilt[2],
                               np.sqrt(adjusted_pose_in_tilt[0] ** 2 + adjusted_pose_in_tilt[1] ** 2))

        if RobotDescription.current_robot_description.name in {"iCub", "tiago_dual"}:
            new_tilt = -new_tilt

        current_pan = robot.get_joint_position(pan_joint)
        current_tilt = robot.get_joint_position(tilt_joint)

        # giskard.avoid_all_collisions()
        giskard.allow_self_collision()
        giskard.achieve_joint_goal({pan_joint: new_pan + current_pan,
                                    tilt_joint: new_tilt + current_tilt})


class DefaultMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real robot while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion):
        pose_in_map = PoseStamped.from_spatial_type(designator.world.transform(designator.target.to_spatial_type(), designator.world.root))
        tip_link = RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame()
        root_link = "map"

        gripper_that_can_collide = designator.arm if designator.allow_gripper_collision else None
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)

        if designator.movement_type == MovementType.STRAIGHT_TRANSLATION:
            giskard.achieve_straight_translation_goal(pose_in_map.position.to_list(), tip_link, root_link)
        elif designator.movement_type == MovementType.STRAIGHT_CARTESIAN:
            giskard.achieve_straight_cartesian_goal(pose_in_map, tip_link, root_link)
        elif designator.movement_type == MovementType.TRANSLATION:
            giskard.achieve_translation_goal(pose_in_map.position.to_list(), tip_link, root_link)
        elif designator.movement_type == MovementType.CARTESIAN:
            giskard.achieve_cartesian_goal(pose_in_map, tip_link, root_link,
                                           grippers_that_can_collide=gripper_that_can_collide)
        if not World.current_world.robot.get_link_pose(tip_link).almost_equal(designator.target, 0.3, 3):
            raise ToolPoseNotReachedError(World.current_world.robot.get_link_pose(tip_link), designator.target)


class DefaultMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real robot to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: 'MoveArmJointsMotion'):
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        # giskard.avoid_all_collisions()
        giskard.allow_self_collision()
        giskard.achieve_joint_goal(joint_goals)


class DefaultMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion):
        name_to_position = dict(zip(designator.names, designator.positions))
        align = designator.align
        tip_link = designator.tip_link
        tip_normal = designator.tip_normal
        root_normal = designator.root_normal
        root_link = designator.root_link
        # giskard.avoid_all_collisions()
        giskard.allow_self_collision()
        giskard.achieve_joint_goal(name_to_position)


class DefaultMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real robot, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion):
        raise NotImplementedError(f"There is DefaultMoveGripperReal process module")


class DefaultOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion):
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class DefaultCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion):
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class DefaultMoveTCPWaypointsReal(ProcessModule):
    """
    Moves the tool center point of the real robot along a list of waypoints while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPWaypointsMotion):
        waypoints = [designator.world.transform(x.to_spatial_type(), designator.world.root) for x in designator.waypoints]
        tip_link = RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame()
        root_link = "map"

        giskard.avoid_all_collisions()
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)

        giskard.achieve_cartesian_waypoints_goal(waypoints=waypoints,
                                                 tip_link=tip_link, root_link=root_link,
                                                 enforce_final_orientation=True if designator.movement_type == WaypointsMovementType.ENFORCE_ORIENTATION_FINAL_POINT else False)


class DefaultManager(ManagerBase):

    def __init__(self):
        super().__init__("default")

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveArmJointsReal(self._move_arm_joints_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultClose(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultCloseReal(self._close_lock)

    def move_tcp_waypoints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCPWaypoints(self._move_tcp_waypoints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveTCPWaypointsReal(self._move_tcp_waypoints_lock)


# Initialize the default manager and register it with the ProcessModuleManager
DefaultManager()
