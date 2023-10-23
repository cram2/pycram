import rospy
import actionlib

from robokudo_msgs.msg import ObjectDesignator as robokudo_ObjetDesignator
from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult
from ..designators.object_designator import ObjectDesignatorDescription, RealObject


def msg_from_obj_desig(obj_desc: ObjectDesignatorDescription) -> robokudo_ObjetDesignator:
    """
    Creates a RoboKudo Object designator from a PyCRAM Object Designator description

    :param obj_desc: The PyCRAM Object designator that should be converted
    :return: The RobotKudo Object Designator for the given PyCRAM designator
    """
    obj_msg = robokudo_ObjetDesignator()
    obj_msg.uid = str(id(obj_desc))
    obj_msg.type = obj_desc.types[0] # For testing purposes

    return obj_msg


def make_query_goal_msg(obj_desc: ObjectDesignatorDescription) -> QueryGoal:
    """
    Creates a QueryGoal message from a PyCRAM Object designator description for the use of Querying RobotKudo.

    :param obj_desc: The PyCRAM object designator description that should be converted
    :return: The RoboKudo QueryGoal for the given object designator description
    """
    goal_msg = QueryGoal()
    goal_msg.obj.uid = str(id(obj_desc))
    goal_msg.obj.type = obj_desc.types[0] # For testing purposes
    return goal_msg


def query(object_desc: ObjectDesignatorDescription) -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
    For sending the query to RoboKudo a simple action client will be created and the Object designator description is
    sent as a goal.

    :param object_desc: The object designator description which describes the object that should be perceived
    :return: An object designator for the found object, if there was an object that fitted the description.
    """
    def active_callback():
        rospy.loginfo("Send query to Robokudo")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        print(result)
        print(state)
        result_desigs = []
        for obj_desig in result:
            for p  in obj_desig.pose:
                result_desigs.append(RealObject.Object("perceived Object", obj_desig.type, None, p))
        return result_desigs

    def feedback_callback(msg):
        print(msg)

    object_goal = make_query_goal_msg(object_desc)
    print(object_goal)

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    wait = client.wait_for_result()
