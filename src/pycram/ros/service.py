import rosservice
import rospy


def get_service_proxy(topic_name: str, service_message) -> rospy.ServiceProxy:
    return rospy.ServiceProxy(topic_name, service_message)


def wait_for_service(topic_name: str):
    rospy.loginfo_once(f"Waiting for service: {topic_name}")
    rospy.wait_for_service(topic_name)
