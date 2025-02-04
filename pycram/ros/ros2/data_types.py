import builtin_interfaces.msg
import rclpy
from . import node
import builtin_interfaces

class Time(builtin_interfaces.msg.Time):
    def __init__(self, time=0.0):
        super().__init__(sec=time)

    @classmethod
    def now(cls):
        return cls(node.get_clock().now().seconds_nanoseconds()[0])
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