import rospy

def create_publisher(topic, msg_type, queue_size=10) -> rospy.Publisher:
    return rospy.Publisher(topic, msg_type, queue_size=queue_size)