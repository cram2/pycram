import rospy
import actionlib
from tmc_msgs.msg import Voice

from ..designator import ObjectDesignatorDescription
from ..datastructures.pose import Pose
from ..worlds.bullet_world import BulletWorld
from ..datastructures.enums import ObjectType
from typing import Any
from geometry_msgs.msg import PoseStamped, PointStamped

is_init = False


def init_robokudo_interface():
    """
    Initializes the RoboKudo interface if it has not been initialized yet.
    """
    global is_init
    if is_init:
        return
    try:
        from robokudo_msgs.msg import ObjectDesignator as robokudo_ObjectDesignator
        from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult
        is_init = True
        rospy.loginfo("Successfully initialized RoboKudo interface")
    except ModuleNotFoundError as e:
        rospy.logwarn("Could not import RoboKudo messages, RoboKudo interface could not be initialized")


def msg_from_obj_desig(obj_desc: ObjectDesignatorDescription) -> 'robokudo_ObjectDesignator':
    """
    Creates a RoboKudo Object designator from a PyCRAM Object Designator description.

    :param obj_desc: The PyCRAM Object designator that should be converted.
    :return: The RoboKudo Object Designator for the given PyCRAM designator.
    """
    obj_msg = robokudo_ObjectDesignator()
    obj_msg.uid = str(id(obj_desc))
    obj_msg.type = obj_desc.types[0]  # For testing purposes

    return obj_msg


def make_query_goal_msg(obj_desc: ObjectDesignatorDescription) -> 'QueryGoal':
    """
    Creates a QueryGoal message from a PyCRAM Object designator description for querying RoboKudo.

    :param obj_desc: The PyCRAM object designator description that should be converted.
    :return: The RoboKudo QueryGoal for the given object designator description.
    """
    from robokudo_msgs.msg import QueryGoal

    goal_msg = QueryGoal()
    goal_msg.obj.type = str(obj_desc.types[0].name)
    return goal_msg


def query(object_desc: ObjectDesignatorDescription) -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
    Creates an action client to send the Object designator description as a goal.

    :param object_desc: The object designator description which describes the object that should be perceived.
    :return: Pose candidates of the found objects.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to RoboKudo")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result

    def feedback_callback(msg):
        pass

    object_goal = make_query_goal_msg(object_desc)

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    client.wait_for_result()

    pose_candidates = {}
    for i in range(0, len(query_result.res[0].pose)):
        pose = Pose.from_pose_stamped(query_result.res[0].pose[i])
        source = query_result.res[0].pose_source[0]
        pose_candidates[source] = pose

    return pose_candidates


def query_empty() -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for all objects.
    Creates an action client to send the Object designator description as a goal.

    :return: query_result of RoboKudo message.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to RoboKudo")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result

    def feedback_callback(msg):
        pass

    object_goal = QueryGoal()
    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    client.wait_for_result()

    return query_result


def query_region(region: str) -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for regions.
    Creates an action client to send the Object designator description as a goal.

    :param region: The specific region to look for, e.g "suturo_couch".
    :return: query_result of RoboKudo message.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to RoboKudo to scan a specific region")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result.res

    def feedback_callback(msg):
        pass

    object_goal = QueryGoal()
    object_goal.obj.location = str(region)
    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    client.wait_for_result()

    return query_result


def query_human() -> PointStamped:
    """
    Sends a query to RoboKudo to locate a human and returns a PointStamped object representing the pose where the human is located.
    Continuously publishes the pose onto the topic /human_pose.

    :return: A PointStamped object containing the pose of the detected human.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global human_bool
    global query_result
    global human_pose

    def active_callback():
        rospy.loginfo("Send query to RoboKudo to perceive a human")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        global human_bool
        human_bool = True
        global query_result
        query_result = result

    def feedback_callback(msg):
        global feedback_result
        feedback_result = msg

    def callback(pose):
        global human_bool
        global human_pose
        human_bool = True
        human_pose = pose

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    object_goal = QueryGoal()
    object_goal.obj.type = 'human'
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)

    human_bool = False
    rospy.Subscriber("/human_pose", PointStamped, callback)

    while not human_bool:
        rospy.sleep(0.5)

    return human_pose


def stop_query_human():
    """
    Sends a query to RoboKudo to stop human detection.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    client.wait_for_server()
    client.cancel_all_goals()
    rospy.loginfo("Cancelled current goal")


def seat_query_human(seat: str) -> Any:
    """
    Sends a query to RoboKudo to check if a place is free to sit.

    :param seat: Name of the seat/region to be checked.
    :return: query_result of RoboKudo message.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to RoboKudo to scan for seat and human")

    def done_callback(state, result: QueryResult):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result.res

    object_goal = QueryGoal()
    object_goal.obj.location = str(seat)

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback)
    client.wait_for_result()

    return query_result
