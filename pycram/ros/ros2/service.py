import rclpy.client

from . import node
from .logging import loginfo_once

services = {}

def get_service_proxy(topic_name: str, service_message) -> rclpy.client.Client:
    if topic_name not in services:
        services[topic_name] = node.create_client(topic_name, service_message)
    return services[topic_name]

def wait_for_service(topic_name: str):
    loginfo_once(f"Waiting for service: {topic_name}")
    services[topic_name].wait_for_service()