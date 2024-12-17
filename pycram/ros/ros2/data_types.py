import rclpy

def Time(time=0.0):
    return rclpy.time.Time(time)

def Duration(duration=0.0):
    return rclpy.durarion.Duration(duration)

def Rate(rate):
    return rate