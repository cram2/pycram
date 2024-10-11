import rospy

from rospy import ServiceException
def Time(time=0.0):
    return rospy.Time(time)

def Duration(duration=0.0):
    return rospy.Duration(duration)

def Rate(rate):
    return rospy.Rate(rate)