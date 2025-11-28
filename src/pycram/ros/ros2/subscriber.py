import rclpy


def create_subscriber(
    topic, msg_type, callback, node, queue_size=10
) -> rclpy.subscription.Subscription:
    node.create_subscription(msg_type, topic, callback, queue_size)
