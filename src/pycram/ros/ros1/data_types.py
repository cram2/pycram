import rospy

from rospy import ServiceException
def Time(time=0.0, nsecs=0):
    return rospy.Time(time, nsecs=nsecs)

def Duration(duration=0.0):
    return rospy.Duration.from_sec(duration)

def Rate(rate):
    return rospy.Rate(rate)