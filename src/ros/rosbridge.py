import os
from threading import Lock
from urllib.parse import urlparse

import roslibpy


class SingletonMeta(type):
    """
    This is a thread-safe implementation of Singleton.
    """

    _instances = {}

    _lock: Lock = Lock()
    """
    We now have a lock object that will be used to synchronize threads during
    first access to the Singleton.
    """

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        # Now, imagine that the program has just been launched. Since there's no
        # Singleton instance yet, multiple threads can simultaneously pass the
        # previous conditional and reach this point almost at the same time. The
        # first of them will acquire lock and will proceed further, while the
        # rest will wait here.
        with cls._lock:
            # The first thread to acquire the lock, reaches this conditional,
            # goes inside and creates the Singleton instance. Once it leaves the
            # lock block, a thread that might have been waiting for the lock
            # release may then enter this section. But since the Singleton field
            # is already initialized, the thread won't create a new object.
            if cls not in cls._instances:
                instance = super().__call__(*args, **kwargs)
                cls._instances[cls] = instance
        return cls._instances[cls]


class ROSBridge(metaclass=SingletonMeta):
    def __init__(self):
        ros_host, ros_port = self.get_ros_master_host_and_port()
        self.ros_client = roslibpy.Ros(ros_host, ros_port)

    @staticmethod
    def get_ros_master_host_and_port():
        uri = urlparse(os.environ["ROS_MASTER_URI"])
        return uri.hostname, 9090

    def __enter__(self):
        self.ros_client.run()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.ros_client.terminate()
