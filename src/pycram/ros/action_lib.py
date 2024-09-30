import actionlib


def create_action_client(topic_name: str, action_message) -> actionlib.SimpleActionClient:
    return actionlib.SimpleActionClient(topic_name, action_message)

