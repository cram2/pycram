import rospy

# Here so it can be imported from pycram.ros
from rospy import ServiceException


def Time(time: int = 0.0, nsecs: int = 0) -> rospy.Time:
    """
    Wrapper for rospy.Time to create a Time object.

    :param time: Time in seconds
    :param nsecs: Time in nanoseconds
    :return: Rospy Time object representing the given time
    """
    return rospy.Time(time, nsecs=nsecs)


def Duration(duration: float = 0.0) -> rospy.Duration:
    """
    Wrapper for rospy.Duration to create a Duration object.

    :param duration: Duration in seconds
    :return: A rospy Duration object representing the given duration
    """
    return rospy.Duration.from_sec(duration)


def Rate(rate: float) -> rospy.Rate:
    """
    Wrapper for rospy.Rate to create a Rate object.

    :param rate: Rate in Hz
    :return: A rospy Rate object representing the given rate
    """
    return rospy.Rate(rate)
