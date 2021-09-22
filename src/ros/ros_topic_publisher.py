import logging
import threading
from abc import ABC, abstractmethod

from ros.rosbridge import ros_client


class ROSTopicPublisher(ABC):
    def __init__(self):
        self.thread = None
        self.kill_event = threading.Event()

    def start_publishing(self):
        logging.info(f"{self.__class__.__name__}::start_publishing: Starting publisher thread...")
        if not ros_client.is_connected:
            raise RuntimeError(f"{self.__class__.__name__}: Cannot start publishing, ROS client not connected")
        if self.kill_event.is_set():
            self.kill_event.clear()
        self.thread = threading.Thread(target=self._publish)
        self.thread.start()
        logging.info(f"{self.__class__.__name__}::start_publishing: Publisher thread started")

    def stop_publishing(self):
        logging.info(f"{self.__class__.__name__}::stop_publishing: Stopping publisher thread...")
        if self.thread:
            self.kill_event.set()
            self.thread.join()
            self.thread = None
            logging.info(f"{self.__class__.__name__}::stop_publishing: Publisher thread stopped")
        else:
            logging.info(f"{self.__class__.__name__}::stop_publishing: Publisher thread not running")

    def __enter__(self):
        self.start_publishing()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_publishing()

    @abstractmethod
    def _publish(self):
        pass