def create_publisher(topic, msg_type, queue_size=10, latch: bool = False) -> None:
    """
    Create a ROS publisher.

    :param topic: The name of the topic to publish to.
    :param msg_type: The type of message to publish.
    :param queue_size: The size of the message queue.
    :param latch: Whether to latch the last message sent.
    :return: A ROS publisher object.
    """
    # This is a placeholder implementation. In a real implementation, you would use
    # the appropriate ROS API to create the publisher.
    raise NotImplementedError("ROS is not available in this environment. ")