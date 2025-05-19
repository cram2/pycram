import builtin_interfaces.msg
import rclpy
from . import node
import builtin_interfaces

class Time(builtin_interfaces.msg.Time):
    """
    Class to abstract the ROS2 Time, to make it more consistent with the ROS1 Time class.
    """
    def __init__(self, time=0, nsecs=0):
        super().__init__(sec=time)

    @classmethod
    def now(cls):
        return builtin_interfaces.msg.Time(**dict(zip(["sec", "nanosec"], node.get_clock().now().seconds_nanoseconds())))
        # return cls(*node.get_clock().now().seconds_nanoseconds())
        #return node.get_clock().now()

    def to_sec(self):
        return self.sec()[0]

# def Time(time=0.0):
#
#     return rclpy.time.Time(nanoseconds=time)

def Duration(duration=0.0):
    # return rclpy.duration.Duration(seconds=duration)
    return builtin_interfaces.msg.Duration(sec=duration)

def Rate(rate):
    return rate