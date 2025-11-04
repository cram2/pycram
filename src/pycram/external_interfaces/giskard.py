import json
import threading

import sys

from semantic_digital_twin.world_description.world_entity import Body

from ..ros import Time
from ..logging import logwarn, loginfo_once, loginfo
from ..ros import get_node_names

from ..datastructures.enums import JointType, ObjectType, Arms
from ..datastructures.pose import PoseStamped
from ..ros import get_service_proxy
from ..robot_description import RobotDescription

from typing_extensions import List, Dict, Callable, Optional
from threading import Lock, RLock
from ..ros import node



giskard_wrapper = None
giskard_update_service = None
is_init = False

number_of_par_goals = 0
giskard_lock = Lock()
giskard_rlock = RLock()
with giskard_rlock:
    par_threads = {}
    par_motion_goal = {}


def thread_safe(func: Callable) -> Callable:
    """
    Adds thread safety to a function via a decorator. This uses the giskard_lock

    :param func: Function that should be thread safe
    :return: A function with thread safety
    """

    def wrapper(*args, **kwargs):
        with giskard_rlock:
            return func(*args, **kwargs)

    return wrapper


def init_giskard_interface(func: Callable) -> Callable:
    """
    Checks if the ROS messages are available and if giskard is running, if that is the case the interface will be
    initialized.

    :param func: Function this decorator should be wrapping
    :return: A callable function which initializes the interface and then calls the wrapped function
    """

    def wrapper(*args, **kwargs):

        from giskardpy_ros.python_interface.old_python_interface import OldGiskardWrapper as GiskardWrapper
        from giskard_msgs.msg import WorldBody, CollisionEntry
        from geometry_msgs.msg import PoseStamped as ROSPoseStamped, PointStamped, QuaternionStamped, Vector3Stamped

        global giskard_wrapper
        global giskard_update_service
        global is_init
        if is_init and "giskard" in get_node_names():
            return func(*args, **kwargs)
        elif is_init and "giskard" not in get_node_names():
            logwarn("Giskard node is not available anymore, could not initialize giskard interface")
            is_init = False
            giskard_wrapper = None
            return

        if "giskard_msgs" not in sys.modules:
            logwarn("Could not initialize the Giskard interface since the giskard_msgs are not imported")
            return

        if "giskard" in get_node_names():
            giskard_wrapper = GiskardWrapper(node)
            loginfo_once("Successfully initialized Giskard interface")
            is_init = True
        else:
            logwarn("Giskard is not running, could not initialize Giskard interface")
            return
        return func(*args, **kwargs)
    return wrapper


# Believe state management between pycram and giskard


@init_giskard_interface
def initial_adding_objects() -> None:
    """
    Adds object that are loaded in the World to the Giskard belief state, if they are not present at the moment.
    """
    groups = giskard_wrapper.get_group_names()
    for obj in World.current_world.objects:
        if obj is World.robot or obj is World.current_world.get_prospection_object_for_object(World.robot):
            continue
        name = obj.name
        if name not in groups:
            print(obj)
            spawn_object(obj)


@init_giskard_interface
def removing_of_objects() -> None:
    """
    Removes objects that are present in the Giskard belief state but not in the World from the Giskard belief state.
    """
    groups = giskard_wrapper.get_group_names()
    object_names = list(
        map(lambda obj: object_names.name, World.current_world.objects))
    diff = list(set(groups) - set(object_names))
    for grp in diff:
        giskard_wrapper.remove_group(grp)


@init_giskard_interface
def sync_worlds(projection: bool = False) -> None:
    """
    Synchronizes the World and the Giskard belief state, this includes adding and removing objects to the Giskard
    belief state such that it matches the objects present in the World and moving the robot to the position it is
    currently at in the World.

    :param projection: Whether the sync in projection mode or reality.
    """
    add_gripper_groups()
    world_object_names = set()
    for obj in World.current_world.objects:
        if obj.name != RobotDescription.current_robot_description.name and obj.obj_type != ObjectType.ROBOT and len(obj.link_name_to_id) != 1:
            world_object_names.add(obj.name)
        if obj.name == RobotDescription.current_robot_description.name or obj.obj_type == ObjectType.ROBOT:
            joint_config = obj.get_positions_of_all_joints()
            non_fixed_joints = list(filter(lambda joint: joint.type != JointType.FIXED and not joint.is_virtual,
                                           obj.joints.values()))
            joint_config_filtered = {joint.name: joint_config[joint.name] for joint in non_fixed_joints}

            if projection:
                giskard_wrapper.monitors.add_set_seed_configuration(joint_config_filtered,
                                                                    RobotDescription.current_robot_description.name)
                giskard_wrapper.monitors.add_set_seed_odometry(obj.get_pose().ros_message(),
                                                               RobotDescription.current_robot_description.name)
    giskard_object_names = set(giskard_wrapper.get_group_names())
    robot_name = {RobotDescription.current_robot_description.name}
    if not world_object_names.union(robot_name).issubset(giskard_object_names):
        giskard_wrapper.clear_world()
    initial_adding_objects()


@init_giskard_interface
def update_pose(object: Body) -> 'UpdateWorldResponse':
    """
    Sends an update message to giskard to update the object position. Might not work when working on the real robot just
    in standalone mode.

    :param object: Object that should be updated
    :return: An UpdateWorldResponse
    """
    return giskard_wrapper.update_group_pose(object.name)


@init_giskard_interface
def spawn_object(object: Body) -> None:
    """
    Spawns a World Object in the giskard belief state.

    :param object: World object that should be spawned
    """
    if len(object.link_name_to_id) == 1:
        geometry = object.get_link_geometry(object.root_link.name)
        if isinstance(geometry, MeshVisualShape):
            filename = geometry.file_name
            spawn_mesh(object.name, filename, object.get_pose())
    else:
        ww = spawn_urdf(object.name, object.path, object.get_pose())
        log.loginfo("GiskardSpawnURDF Return value: {} ObjectName:{}".format(ww,object.name))


@init_giskard_interface
def remove_object(object: Body) -> 'UpdateWorldResponse':
    """
    Removes an object from the giskard belief state.

    :param object: The World Object that should be removed
    """
    return giskard_wrapper.remove_group(object.name)


@init_giskard_interface
def spawn_urdf(name: str, urdf_path: str, pose: PoseStamped) -> 'UpdateWorldResponse':
    """
    Spawns an URDF in giskard's belief state.

    :param name: Name of the URDF
    :param urdf_path: Path to the URDF file
    :param pose: Pose in which the URDF should be spawned
    :return: An UpdateWorldResponse message
    """
    urdf_string = ""
    with open(urdf_path) as f:
        urdf_string = f.read()

    return giskard_wrapper.add_urdf(name, urdf_string, pose)


@init_giskard_interface
def spawn_mesh(name: str, path: str, pose: 'PoseStamped') -> 'UpdateWorldResponse':
    """
    Spawns a mesh into giskard's belief state

    :param name: Name of the mesh
    :param path: Path to the mesh file
    :param pose: Pose in which the mesh should be spawned
    :return: An UpdateWorldResponse message
    """
    return giskard_wrapper.add_mesh(name, path, pose)


# Sending Goals to Giskard

@thread_safe
def _manage_par_motion_goals(goal_func, *args) -> Optional['MoveResult']:
    """
    Manages multiple goals that should be executed in parallel. The current sequence of motion goals is saved and the
    parallel motion goal is loaded if there is one, then the new motion goal given by ``goal_func`` is added to the
    parallel motion goal. If this was the last motion goal for the parallel motion goal it is then executed.

    :param goal_func: Function which adds a new motion goal to the giskard_wrapper
    :param args: Arguments for the ``goal_func`` function
    :return: MoveResult of the execution if there was an execution, True if a new motion goal was added to the giskard_wrapper and None in any other case
    """
    # key is the instance of the parallel language element, value is a list of threads that should be executed in
    # parallel
    for key, value in par_threads.items():
        # if the current thread is in the list of threads that should be executed in parallel backup the current list
        # of motion goals and monitors
        if threading.get_ident() in value:
            tmp_goals = giskard_wrapper.motion_goals.get_goals()
            tmp_monitors = giskard_wrapper.monitors.get_monitors()

            if key in par_motion_goal.keys():
                # giskard_wrapper.cmd_seq = par_motion_goal[key]
                giskard_wrapper.motion_goals._goals = par_motion_goal[key][0]
                giskard_wrapper.monitors._monitors = par_motion_goal[key][1]
            else:
                giskard_wrapper.clear_motion_goals_and_monitors()

            goal_func(*args)

            # Check if there are multiple constraints that use the same joint, if this is the case the
            used_joints = set()
            for cmd in giskard_wrapper.motion_goals.get_goals():
                par_value_pair = json.loads(cmd.kwargs)
                if "tip_link" in par_value_pair.keys() and "root_link" in par_value_pair.keys():
                    if par_value_pair["tip_link"] == RobotDescription.current_robot_description.base_link:
                        continue
                    chain = World.robot.description.get_chain(par_value_pair["root_link"],
                                                              par_value_pair["tip_link"])
                    if set(chain).intersection(used_joints) != set():
                        giskard_wrapper.motion_goals._goals = tmp_goals
                        giskard_wrapper.monitors._monitors = tmp_monitors
                        raise AttributeError(
                            f"The joint(s) {set(chain).intersection(used_joints)} is used by multiple Designators")
                    else:
                        [used_joints.add(joint) for joint in chain]

                elif "goal_state" in par_value_pair.keys():
                    if set(par_value_pair["goal_state"].keys()).intersection(used_joints) != set():
                        giskard_wrapper.motion_goals._goals = tmp_goals
                        giskard_wrapper.monitors._monitors = tmp_monitors
                        raise AttributeError(
                            f"The joint(s) {set(par_value_pair['goal_state'].keys()).intersection(used_joints)} is used by multiple Designators")
                    else:
                        [used_joints.add(joint) for joint in par_value_pair["goal_state"].keys()]

            par_threads[key].remove(threading.get_ident())
            # If this is the last thread that should be executed in parallel, execute the complete sequence of motion
            # goals
            if len(par_threads[key]) == 0:
                if key in par_motion_goal.keys():
                    del par_motion_goal[key]
                del par_threads[key]
                # giskard_wrapper.add_default_end_motion_conditions()
                res = giskard_wrapper.execute()
                giskard_wrapper.motion_goals._goals = tmp_goals
                giskard_wrapper.monitors._monitors = tmp_monitors
                return res
            # If there are still threads that should be executed in parallel, save the current state of motion goals and
            # monitors.
            else:
                par_motion_goal[key] = [giskard_wrapper.motion_goals.get_goals(),
                                        giskard_wrapper.monitors.get_monitors()]
                giskard_wrapper.motion_goals._goals = tmp_goals
                giskard_wrapper.monitors._monitors = tmp_monitors
                return True


@init_giskard_interface
@thread_safe
def achieve_joint_goal(goal_poses: Dict[str, float]) -> 'MoveResult':
    """
    Takes a dictionary of joint position that should be achieved, the keys in the dictionary are the joint names and
    values are the goal joint positions.

    :param goal_poses: Dictionary with joint names and position goals
    :return: MoveResult message for this goal
    """
    set_joint_goal(goal_poses)
    return giskard_wrapper.execute()


@init_giskard_interface
@thread_safe
def set_joint_goal(goal_poses: Dict[str, float]) -> None:
    """
    Takes a dictionary of joint position that should be achieved, the keys in the dictionary are the joint names and
    values are the goal joint positions.

    :param goal_poses: Dictionary with joint names and position goals
    """
    sync_worlds()
    par_return = _manage_par_motion_goals(giskard_wrapper.set_joint_goal, goal_poses)
    if par_return:
        return par_return

    giskard_wrapper.set_joint_goal(goal_poses)


@init_giskard_interface
@thread_safe
def achieve_cartesian_goal(goal_pose: 'PoseStamped', tip_link: str, root_link: str,
                           position_threshold: float = 0.02,
                           orientation_threshold: float = 0.02,
                           use_monitor: bool = True,
                           grippers_that_can_collide: Optional[Arms] = None) -> 'MoveResult':
    """
    Takes a cartesian position and tries to move the tip_link to this position using the chain defined by
    tip_link and root_link.

    :param goal_pose: The position which should be achieved with tip_link
    :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
    :param root_link: The starting link of the chain which should be used to achieve this goal
    :param position_threshold: Position distance at which the goal is successfully reached
    :param orientation_threshold: Orientation distance at which the goal is successfully reached
    :param use_monitor: Whether to use a monitor for this goal or not.
    :param grippers_that_can_collide: The gripper(s) that should be allowed to collide.
    :return: MoveResult message for this goal
    """
    sync_worlds()
    par_return = _manage_par_motion_goals(giskard_wrapper.set_cart_goal, goal_pose.ros_message(),
                                          tip_link, root_link)
    if par_return:
        return par_return

    cart_monitor1 = None
    if use_monitor:
        cart_monitor1 = giskard_wrapper.monitors.add_cartesian_pose(root_link=root_link, tip_link=tip_link,
                                                                    goal_pose=goal_pose.ros_message(),
                                                                    position_threshold=position_threshold, orientation_threshold=orientation_threshold,
                                                                    name='cart goal 1')
        end_monitor = giskard_wrapper.monitors.add_local_minimum_reached(start_condition=cart_monitor1)

    giskard_wrapper.motion_goals.add_cartesian_pose(name='g1', root_link=root_link, tip_link=tip_link,
                                                    goal_pose=goal_pose.ros_message(),
                                                    end_condition=cart_monitor1)

    if use_monitor:
        giskard_wrapper.monitors.add_end_motion(start_condition=end_monitor)

    giskard_wrapper.motion_goals.avoid_all_collisions()
    if grippers_that_can_collide is not None:
        allow_gripper_collision(grippers_that_can_collide)

    return giskard_wrapper.execute()


@init_giskard_interface
@thread_safe
def achieve_straight_cartesian_goal(goal_pose: 'PoseStamped', tip_link: str,
                                    root_link: str,
                                    grippers_that_can_collide: Optional[Arms] = None) -> 'MoveResult':
    """
    Takes a cartesian position and tries to move the tip_link to this position in a straight line, using the chain
    defined by tip_link and root_link.

    :param goal_pose: The position which should be achieved with tip_link
    :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
    :param root_link: The starting link of the chain which should be used to achieve this goal
    :param grippers_that_can_collide: The gripper(s) that should be allowed to collide.
    :return: MoveResult message for this goal
    """
    sync_worlds()
    par_return = _manage_par_motion_goals(giskard_wrapper.set_straight_cart_goal, goal_pose.ros_message(),
                                          tip_link, root_link)
    if par_return:
        return par_return

    giskard_wrapper.avoid_all_collisions()
    if grippers_that_can_collide is not None:
        allow_gripper_collision(grippers_that_can_collide)
    giskard_wrapper.set_straight_cart_goal(goal_pose.ros_message(), tip_link, root_link)
    # giskard_wrapper.add_default_end_motion_conditions()
    return giskard_wrapper.execute()


@init_giskard_interface
@thread_safe
def achieve_translation_goal(goal_point: List[float], tip_link: str, root_link: str) -> 'MoveResult':
    """
    Tries to move the tip_link to the position defined by goal_point using the chain defined by root_link and
    tip_link. Since goal_point only defines the position but no rotation, rotation is not taken into account.

    :param goal_point: The goal position of the tip_link
    :param tip_link: The link which should be moved to goal_point as well as the end of the used chain
    :param root_link: The start link of the chain
    :return: MoveResult message for this goal
    """
    sync_worlds()
    par_return = _manage_par_motion_goals(giskard_wrapper.set_translation_goal, make_point_stamped(goal_point),
                                          tip_link, root_link)
    if par_return:
        return par_return

    giskard_wrapper.set_translation_goal(make_point_stamped(goal_point), tip_link, root_link)
    # giskard_wrapper.add_default_end_motion_conditions()
    return giskard_wrapper.execute()


@init_giskard_interface
@thread_safe
def achieve_straight_translation_goal(goal_point: List[float], tip_link: str, root_link: str) -> 'MoveResult':
    """
    Tries to move the tip_link to the position defined by goal_point in a straight line, using the chain defined by
    root_link and tip_link. Since goal_point only defines the position but no rotation, rotation is not taken into account.

    :param goal_point: The goal position of the tip_link
    :param tip_link: The link which should be moved to goal_point as well as the end of the used chain
    :param root_link: The start link of the chain
    :return: MoveResult message for this goal
    """
    sync_worlds()
    par_return = _manage_par_motion_goals(giskard_wrapper.set_straight_translation_goal,
                                          make_point_stamped(goal_point),
                                          tip_link, root_link)
    if par_return:
        return par_return

    giskard_wrapper.set_straight_translation_goal(make_point_stamped(goal_point), tip_link, root_link)
    # giskard_wrapper.add_default_end_motion_conditions()
    return giskard_wrapper.execute()


@init_giskard_interface
@thread_safe
def achieve_rotation_goal(quat: List[float], tip_link: str, root_link: str) -> 'MoveResult':
    """
    Tries to bring the tip link into the rotation defined by quat using the chain defined by root_link and
    tip_link.

    :param quat: The rotation that should be achieved, given as a quaternion
    :param tip_link: The link that should be in the rotation defined by quat
    :param root_link: The start link of the chain
    :return: MoveResult message for this goal
    """
    sync_worlds()
    par_return = _manage_par_motion_goals(giskard_wrapper.set_rotation_goal, make_quaternion_stamped(quat),
                                          tip_link, root_link)
    if par_threads:
        return par_return

    giskard_wrapper.set_rotation_goal(make_quaternion_stamped(quat), tip_link, root_link)
    # giskard_wrapper.add_default_end_motion_conditions()
    return giskard_wrapper.execute()


@init_giskard_interface
@thread_safe
def achieve_align_planes_goal(goal_normal: List[float], tip_link: str, tip_normal: List[float],
                              root_link: str) -> 'MoveResult':
    """
    Tries to align the plane defined by tip normal with goal_normal using the chain between root_link and
    tip_link.

    :param goal_normal: The goal plane, given as a list of XYZ
    :param tip_link: The end link of the chain that should be used.
    :param tip_normal: The plane that should be aligned with goal_normal, given as a list of XYZ
    :param root_link: The starting link of the chain that should be used.
    :return: MoveResult message for this goal
    """
    sync_worlds()
    par_return = _manage_par_motion_goals(giskard_wrapper.set_align_planes_goal, make_vector_stamped(goal_normal),
                                          tip_link, make_vector_stamped(tip_normal), root_link)
    if par_return:
        return par_return

    giskard_wrapper.set_align_planes_goal(make_vector_stamped(goal_normal), tip_link,
                                          make_vector_stamped(tip_normal),
                                          root_link)
    # giskard_wrapper.add_default_end_motion_conditions()
    return giskard_wrapper.execute()


@init_giskard_interface
@thread_safe
def achieve_open_container_goal(tip_link: str, environment_link: str) -> 'MoveResult':
    """
    Tries to open a container in an environment, this only works if the container was added as a URDF. This goal assumes
    that the handle was already grasped. Can only handle container with 1 DOF

    :param tip_link: The End effector that should open the container
    :param environment_link: The name of the handle for this container.
    :return: MoveResult message for this goal
    """
    sync_worlds()
    par_return = _manage_par_motion_goals(giskard_wrapper.set_open_container_goal, tip_link, environment_link)
    if par_return:
        return par_return
    giskard_wrapper.set_open_container_goal(tip_link, environment_link)
    # giskard_wrapper.add_default_end_motion_conditions()
    return giskard_wrapper.execute()


@init_giskard_interface
@thread_safe
def achieve_close_container_goal(tip_link: str, environment_link: str) -> 'MoveResult':
    """
    Tries to close a container, this only works if the container was added as a URDF. Assumes that the handle of the
    container was already grasped. Can only handle container with 1 DOF.

    :param tip_link: Link name that should be used to close the container.
    :param environment_link: Name of the handle
    :return: MoveResult message for this goal
    """
    sync_worlds()
    par_return = _manage_par_motion_goals(giskard_wrapper.set_close_container_goal, tip_link, environment_link)
    if par_return:
        return par_return

    giskard_wrapper.set_close_container_goal(tip_link, environment_link)
    # giskard_wrapper.add_default_end_motion_conditions()
    return giskard_wrapper.execute()


@init_giskard_interface
def achieve_cartesian_waypoints_goal(waypoints: List['PoseStamped'], tip_link: str,
                                     root_link: str, enforce_final_orientation: bool = True) -> 'MoveResult':
    """
        Tries to achieve each waypoint in the given sequence of waypoints.
        If :param enforce_final_orientation is False, each waypoint needs a corresponding orientation. If it is True only
        the last waypoint needs to have an orientation.

        :param waypoints: The sequence of waypoints as poses to achieve.
        :param tip_link: The endeffector link of the chain that should be used.
        :param root_link: The root link of the chain that should be used.
        :param enforce_final_orientation: If true, only achieve the orientation of the last waypoint. If false, achieve the orientation of each waypoint.
        :return: MoveResult message for this goal
        """
    old_position_monitor = None
    old_orientation_monitor = None

    for i, waypoint in enumerate(waypoints):
        point = make_point_stamped(waypoint.position_as_list())
        orientation = make_quaternion_stamped(waypoint.orientation_as_list())
        start_condition = '' if not old_position_monitor else old_position_monitor

        # -------- Monitor Logic ------------
        if not enforce_final_orientation or (enforce_final_orientation and i == len(waypoints) - 1):
            if not enforce_final_orientation:
                start_condition = '' if not old_orientation_monitor else f'{old_orientation_monitor} and {old_position_monitor}'
            orientation_monitor = giskard_wrapper.monitors.add_cartesian_orientation(goal_orientation=orientation,
                                                                     tip_link=tip_link, root_link=root_link,
                                                                     start_condition=start_condition,
                                                                     name=str(id(waypoint)) + 'orientation')
            old_orientation_monitor = orientation_monitor

        # in all cases a position monitor is needed for each waypoint
        position_monitor = giskard_wrapper.monitors.add_cartesian_position(goal_point=point, tip_link=tip_link,
                                                            root_link=root_link,
                                                            start_condition=start_condition, name=str(id(waypoint)),
                                                            threshold=0.01 + (0.01 * (len(waypoints) - 1 - i)))
        # -------- Task Logic ---------------
        task_end_condition = position_monitor
        if not enforce_final_orientation or (enforce_final_orientation and i == len(waypoints) - 1):
            task_end_condition = f'{orientation_monitor} and {position_monitor}'
            giskard_wrapper.motion_goals.add_cartesian_orientation(goal_orientation=orientation,
                                                                       tip_link=tip_link, root_link=root_link,
                                                                       end_condition=task_end_condition,
                                                                       start_condition=start_condition,
                                                                       name=str(id(waypoint)) + 'orientation')

        # in all cases a position goal is needed for each waypoint
        giskard_wrapper.motion_goals.add_cartesian_position(goal_point=point, tip_link=tip_link,
                                                            root_link=root_link,
                                                            end_condition=task_end_condition, start_condition=start_condition,
                                                            name=str(id(waypoint)))


        old_position_monitor = position_monitor


    giskard_wrapper.monitors.add_end_motion(start_condition=f'{old_position_monitor} and {old_orientation_monitor}')
    giskard_wrapper.monitors.add_max_trajectory_length(30)
    giskard_wrapper.execute(add_default=False)


# Projection Goals


@init_giskard_interface
def projection_cartesian_goal(goal_pose: 'PoseStamped', tip_link: str, root_link: str) -> 'MoveResult':
    """
    Tries to move the tip_link to the position defined by goal_pose using the chain defined by tip_link and root_link.
    The goal_pose is projected to the closest point on the robot's workspace.

    :param goal_pose: The position which should be achieved with tip_link
    :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
    :param root_link: The starting link of the chain which should be used to achieve this goal
    :return: MoveResult message for this goal
    """
    sync_worlds(projection=True)
    giskard_wrapper.set_cart_goal(goal_pose.ros_message(), tip_link, root_link)
    return giskard_wrapper.projection()


@init_giskard_interface
def projection_cartesian_goal_with_approach(approach_pose: 'PoseStamped', goal_pose: 'PoseStamped', tip_link: str, root_link: str,
                                            robot_base_link: str) -> 'MoveResult':
    """
    Tries to achieve the goal_pose using the chain defined by tip_link and root_link. The approach_pose is used to drive
    the robot to a pose close the actual goal pose, the robot_base_link is used to define the base link of the robot.

    :param approach_pose: Pose near the goal_pose
    :param goal_pose: Pose to which the tip_link should be moved
    :param tip_link: The link which should be moved to goal_pose, usually the tool frame
    :param root_link: The start of the link chain which should be used for planning
    :param robot_base_link: The base link of the robot
    :return: A trajectory calculated to move the tip_link to the goal_pose
    """
    sync_worlds(projection=True)
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.set_cart_goal(approach_pose.ros_message(), robot_base_link, "map")
    giskard_wrapper.projection()
    giskard_wrapper.avoid_all_collisions()
    giskard_wrapper.set_cart_goal(goal_pose.ros_message(), tip_link, root_link)
    return giskard_wrapper.projection()


@init_giskard_interface
def projection_joint_goal(goal_poses: Dict[str, float], allow_collisions: bool = False) -> 'MoveResult':
    """
    Tries to achieve the joint goal defined by goal_poses, the goal_poses are projected to the closest point on the
    robot's workspace.

    :param goal_poses: Dictionary with joint names and position goals
    :param allow_collisions: If all collisions should be allowed for this goal
    :return: MoveResult message for this goal
    """
    sync_worlds(projection=True)
    if allow_collisions:
        giskard_wrapper.allow_all_collisions()
    giskard_wrapper.set_joint_goal(goal_poses)
    return giskard_wrapper.projection()


# Managing collisions

@init_giskard_interface
def allow_gripper_collision(gripper: Arms, at_goal: bool = False) -> None:
    """
    Allows the specified gripper to collide with anything.

    :param gripper: The gripper which can collide, either 'Arms.RIGHT', 'Arms.LEFT' or 'Arms.BOTH'
    :param at_goal: If the collision should be allowed only for this motion goal.
    """
    add_gripper_groups()
    for gripper_group in get_gripper_group_names():
        if gripper.name.lower() in gripper_group or gripper == Arms.BOTH:
            if at_goal:
                giskard_wrapper.motion_goals.allow_collision(gripper_group, CollisionEntry.ALL)
            else:
                giskard_wrapper.allow_collision(gripper_group, CollisionEntry.ALL)


@init_giskard_interface
def get_gripper_group_names() -> List[str]:
    """
    :return: The list of groups that are registered in giskard which have 'gripper' in their name.
    """
    groups = giskard_wrapper.get_group_names()
    return list(filter(lambda elem: "gripper" in elem, groups))


@init_giskard_interface
def add_gripper_groups() -> None:
    """
    Add the gripper links as a group for collision avoidance.

    :return: Response of the RegisterGroup Service
    """
    with giskard_lock:
        for name in giskard_wrapper.get_group_names():
            if "gripper" in name:
                return
        for description in RobotDescription.current_robot_description.get_manipulator_chains():
            giskard_wrapper.register_group(description.name + "_gripper", description.end_effector.start_link,
                                           RobotDescription.current_robot_description.name)


@init_giskard_interface
def avoid_all_collisions() -> None:
    """
    Will avoid all collision for the next goal.
    """
    giskard_wrapper.avoid_all_collisions()


@init_giskard_interface
def allow_self_collision() -> None:
    """
    Will allow the robot collision with itself.
    """
    giskard_wrapper.allow_self_collision()


@init_giskard_interface
def avoid_collisions(object1: Body, object2: Body) -> None:
    """
    Will avoid collision between the two objects for the next goal.

    :param object1: The first World Object
    :param object2: The second World Object
    """
    giskard_wrapper.avoid_collision(-1, object1.name, object2.name)


# Creating ROS messages

@init_giskard_interface
def make_world_body(object: Body) -> 'WorldBody':
    """
    Create a WorldBody message for a World Object. The WorldBody will contain the URDF of the World Object

    :param object: The World Object
    :return: A WorldBody message for the World Object
    """
    urdf_string = ""
    with open(object.path) as f:
        urdf_sting = f.read()
    urdf_body = WorldBody()
    urdf_body.type = WorldBody.URDF_BODY
    urdf_body.urdf = urdf_string

    return urdf_body


def make_point_stamped(point: List[float]) -> 'PointStamped':
    """
    Creates a PointStamped message for the given position in world coordinate frame.

    :param point: XYZ coordinates of the point
    :return: A PointStamped message
    """
    msg = PointStamped()
    msg.header.stamp = Time().now()
    msg.header.frame_id = "map"

    msg.point.x = point[0]
    msg.point.y = point[1]
    msg.point.z = point[2]

    return msg


def make_quaternion_stamped(quaternion: List[float]) -> 'QuaternionStamped':
    """
    Creates a QuaternionStamped message for the given quaternion.

    :param quaternion: The quaternion as a list of xyzw
    :return: A QuaternionStamped message
    """
    msg = QuaternionStamped()
    msg.header.stamp = Time().now()
    msg.header.frame_id = "map"

    msg.quaternion.x = quaternion[0]
    msg.quaternion.y = quaternion[1]
    msg.quaternion.z = quaternion[2]
    msg.quaternion.w = quaternion[3]

    return msg


def make_vector_stamped(vector: List[float]) -> 'Vector3Stamped':
    """
    Creates a Vector3Stamped message, this is similar to PointStamped but represents a vector instead of a point.

    :param vector: The vector given as xyz in world frame
    :return: A Vector3Stamped message
    """
    msg = Vector3Stamped()
    msg.header.stamp = Time().now()
    msg.header.frame_id = "map"

    msg.vector.x = vector[0]
    msg.vector.y = vector[1]
    msg.vector.z = vector[2]

    return msg
