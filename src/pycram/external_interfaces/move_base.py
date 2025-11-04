import sys

from ..ros import  create_action_client
from ..logging import  logwarn, loginfo
from ..ros import  get_node_names

from typing import Callable


# Global variables for shared resources
nav_action_client = None
is_init = False


def create_nav_action_client():
    """Creates a new action client for the move_base interface."""
    client = create_action_client("move_base", MoveBaseAction)
    loginfo("Waiting for move_base action server")
    client.wait_for_server()
    return client


def init_nav_interface(func: Callable) -> Callable:
    """Ensures initialization of the navigation interface before function execution."""

    def wrapper(*args, **kwargs):
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from geometry_msgs.msg import PoseStamped


        global is_init
        global nav_action_client

        if is_init:
            return func(*args, **kwargs)

        if "move_base_msgs" not in sys.modules:
            logwarn("Could not initialize the navigation interface: move_base_msgs not imported")
            return

        if "/move_base" in get_node_names():
            nav_action_client = create_nav_action_client()
            loginfo("Successfully initialized navigation interface")
            is_init = True
        else:
            logwarn("Move_base is not running, could not initialize navigation interface")
            return

        return func(*args, **kwargs)

    return wrapper


@init_nav_interface
def query_pose_nav(navpose: 'PoseStamped'):
    """Sends a goal to the move_base service, initiating robot navigation to a given pose."""
    global nav_action_client
    global query_result

    def active_callback():
        loginfo("Sent query to move_base")

    def done_callback(state, result):
        loginfo("Finished moving")
        global query_result
        query_result = result

    goal_msg = MoveBaseGoal()
    goal_msg.target_pose = navpose
    nav_action_client.send_goal(goal_msg, active_cb=active_callback, done_cb=done_callback)

    nav_action_client.wait_for_result()

    return query_result


def cancel_nav():
    """Cancels the current navigation goal."""
    global nav_action_client
    nav_action_client.cancel_all_goals()
