import rospy
import actionlib

from robokudo_msgs.msg import ObjectDesignator as robokudo_ObjetDesignator
from robokudo_msgs.msg import QueryAction, QueryGoal
from ..bullet_world import Object
from ..designators.object_designator import ObjectDesignatorDescription


def msg_from_obj_desig(obj_desc: ObjectDesignatorDescription):
    obj_msg = robokudo_ObjetDesignator()
    obj_msg.uid = str(id(obj_desc))
    obj_msg.type = obj_desc.types[0] # For testing purposes

    return obj_msg


def make_query_goal_msg(obj_desc: ObjectDesignatorDescription):
    goal_msg = QueryGoal()
    goal_msg.obj.uid = str(id(obj_desc))
    goal_msg.obj.type = obj_desc.types[0] # For testing purposes
    return goal_msg


def query(object_desc: ObjectDesignatorDescription):
    def active_callback():
        rospy.loginfo("Send query to Robokudo")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        print(result)
        print(state)

    def feedback_callback(msg):
        print(msg)

    #object_desig = msg_from_obj_desig(object_desc)
    object_goal = make_query_goal_msg(object_desc)
    print(object_goal)

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    wait = client.wait_for_result()
