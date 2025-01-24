import rclpy
from . import node

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
    return rclpy.duration.Duration(seconds=duration)

def Rate(rate):
    return rate