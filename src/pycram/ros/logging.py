import rospy


def logwarn(message: str):
    rospy.logwarn(message)


def loginfo(message: str):
    rospy.loginfo(message)


def logerr(message: str):
    rospy.logerr(message)


def logdebug(message: str):
    rospy.logdebug(message)


def logwarn_once(message: str):
    rospy.logwarn_once(message)


def loginfo_once(message: str):
    rospy.loginfo_once(message)


def logerr_once(message: str):
    rospy.logerr_once(message)


def logdebug_once(message: str):
    rospy.logdebug_once(message)
