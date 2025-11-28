import rclpy


def create_publisher(topic, msg_type, node, queue_size=10) -> rclpy.publisher.Publisher:
    return node.create_publisher(msg_type, topic, queue_size)
