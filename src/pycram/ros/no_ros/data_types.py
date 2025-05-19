import time
import datetime


class Time:
    """
    Class to abstract the ROS2 Time, to make it more consistent with the ROS1 Time class.
    """
    def __init__(self, time=0.0):
        self.time = time

    @classmethod
    def now(cls):
        return datetime.datetime.now().timestamp()
        #return builtin_interfaces.msg.Time(**dict(zip(["sec", "nanosec"], node.get_clock().now().seconds_nanoseconds())))
        # return cls(*node.get_clock().now().seconds_nanoseconds())
        #return node.get_clock().now()

    def to_sec(self):
        return datetime.datetime.now().timestamp()
        #return self.sec()[0]

def Duration(duration=0.0):
    return duration

def Rate(rate):
    return rate


class ServiceException(Exception):
    """
    Exception class for service exceptions.
    """
    def __init__(self, message: str):
        super().__init__(message)
        self.message = message