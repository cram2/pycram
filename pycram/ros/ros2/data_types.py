import rclpy

def Time(time=0.0):
    return rclpy.time.Time(time)

def Duration(duration=0.0):
    return rclpy.duration.Duration(seconds=duration)

def Rate(rate):
    return rate