import builtin_interfaces.msg
import rclpy
from . import node
import builtin_interfaces

class Time(rclpy.time.Time):
    def __init__(self, time=0.0):
        super().__init__(nanoseconds=time)

    @staticmethod
    def now():
        return node.get_clock().now()

# def Time(time=0.0):
#
#     return rclpy.time.Time(nanoseconds=time)

def Duration(duration=0.0):
    # return rclpy.duration.Duration(seconds=duration)
    return builtin_interfaces.msg.Duration(sec=duration)

def Rate(rate):
    return rate