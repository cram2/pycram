import actionlib

from actionlib import SimpleActionClient


def create_action_client(topic_name: str, action_message) -> SimpleActionClient:
    return actionlib.SimpleActionClient(topic_name, action_message)
