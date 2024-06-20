import sys
from typing_extensions import Callable

import rospy
import actionlib
import rosnode

from ..designator import ObjectDesignatorDescription
from ..datastructures.pose import Pose
from ..local_transformer import LocalTransformer
from ..datastructures.world import World
from ..datastructures.enums import ObjectType

try:
    from robokudo_msgs.msg import ObjectDesignator as robokudo_ObjetDesignator
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult
except ModuleNotFoundError as e:
    rospy.logwarn(f"Could not import RoboKudo messages, RoboKudo interface could not be initialized")

robokudo_action_client = None


def init_robokudo_interface(func: Callable) -> Callable:
    """
    Tries to import the RoboKudo messages and with that initialize the RoboKudo interface.
    """
    def wrapper(*args, **kwargs):
        global robokudo_action_client
        topics = list(map(lambda x: x[0], rospy.get_published_topics()))
        if "robokudo_msgs" not in sys.modules:
            rospy.logwarn("Could not initialize the RoboKudo interface since the robokudo_msgs are not imported")
            return

        if "/robokudo" in rosnode.get_node_names():
            robokudo_action_client = create_robokudo_action_client()
            rospy.loginfo("Successfully initialized robokudo interface")
        else:
            rospy.logwarn("RoboKudo is not running, could not initialize RoboKudo interface")
            return

        return func(*args, **kwargs)
    return wrapper


def create_robokudo_action_client() -> Callable:
    """
    Creates a new action client for the RoboKudo query interface and returns a function encapsulating the action client.
    The returned function can be called with an ObjectDesigantor as parameter and returns the result of the action client.

    :return: A callable function encapsulating the action client
    """
    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()

    def action_client(object_desc):
        global query_result

        def active_callback():
            rospy.loginfo("Send query to Robokudo")

        def done_callback(state, result):
            rospy.loginfo("Finished perceiving")
            global query_result
            query_result = result

        def feedback_callback(msg):
            pass

        object_goal = make_query_goal_msg(object_desc)
        client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
        wait = client.wait_for_result()
        return query_result

    return action_client


def msg_from_obj_desig(obj_desc: ObjectDesignatorDescription) -> 'robokudo_ObjetDesignator':
    """
    Creates a RoboKudo Object designator from a PyCRAM Object Designator description

    :param obj_desc: The PyCRAM Object designator that should be converted
    :return: The RobotKudo Object Designator for the given PyCRAM designator
    """
    obj_msg = robokudo_ObjetDesignator()
    obj_msg.uid = str(id(obj_desc))
    obj_msg.obj_type = obj_desc.types[0] # For testing purposes

    return obj_msg


def make_query_goal_msg(obj_desc: ObjectDesignatorDescription) -> 'QueryGoal':
    """
    Creates a QueryGoal message from a PyCRAM Object designator description for the use of Querying RobotKudo.

    :param obj_desc: The PyCRAM object designator description that should be converted
    :return: The RoboKudo QueryGoal for the given object designator description
    """
    goal_msg = QueryGoal()
    goal_msg.obj.uid = str(id(obj_desc))
    goal_msg.obj.obj_type = str(obj_desc.types[0].name)  # For testing purposes
    if ObjectType.JEROEN_CUP == obj_desc.types[0]:
        goal_msg.obj.color.append("blue")
    elif ObjectType.BOWL == obj_desc.types[0]:
        goal_msg.obj.color.append("red")
    return goal_msg


@init_robokudo_interface
def query(object_desc: ObjectDesignatorDescription) -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
    For sending the query to RoboKudo a simple action client will be created and the Object designator description is
    sent as a goal.

    :param object_desc: The object designator description which describes the object that should be perceived
    :return: An object designator for the found object, if there was an object that fitted the description.
    """
    query_result = robokudo_action_client(object_desc)
    pose_candidates = {}
    if query_result.res == []:
        rospy.logwarn("No suitable object could be found")
        return

    for i in range(0, len(query_result.res[0].pose)):
        pose = Pose.from_pose_stamped(query_result.res[0].pose[i])
        pose.frame = World.current_world.robot.get_link_tf_frame(pose.frame)  # TODO: pose.frame is a link name?
        source = query_result.res[0].poseSource[i]

        lt = LocalTransformer()
        pose = lt.transform_pose(pose, "map")

        pose_candidates[source] = pose

    return pose_candidates
