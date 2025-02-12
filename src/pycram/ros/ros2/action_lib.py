from rclpy.action import ActionClient
from . import node

def create_action_client(topic_name: str, action_message) -> ActionClient:
    return ActionClient(node, action_message, topic_name)