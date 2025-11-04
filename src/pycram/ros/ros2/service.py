import time
import rclpy.client

from . import node
from ...logging import loginfo_once

services = {}

class ServiceProxy:
    def __init__(self, topic_name, service_message):
        self.service = node.create_client(service_message, topic_name)
        self.message_type = service_message
    def __call__(self, *args, **kwargs):
        future = self.service.call_async(*args, **kwargs)
        while not future.done():
            time.sleep(0.1)
        return future.result()

    def wait_for_service(self, *args, **kwargs):
        self.service.wait_for_service()


def get_service_proxy(topic_name: str, service_message) -> rclpy.client.Client:
    service = ServiceProxy(topic_name, service_message)
    services[topic_name] = service
    return service

def wait_for_service(topic_name: str, service_message):
    if not topic_name in services:
        service = get_service_proxy(topic_name, service_message)
    loginfo_once(f"Waiting for service: {topic_name}")
    services[topic_name].wait_for_service()