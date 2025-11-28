import rospy


def create_subscriber(topic, msg_type, callback, queue_size=10) -> rospy.Subscriber:
    return rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)
