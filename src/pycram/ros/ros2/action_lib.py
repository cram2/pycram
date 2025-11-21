from rclpy.action import ActionClient

def create_action_client(topic_name: str, action_message, node) -> ActionClient:
    return ActionClient(node, action_message, topic_name)