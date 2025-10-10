import rclpy.logging
from typing_extensions import TYPE_CHECKING

from . import node
import os

from ...datastructures.enums import LoggerLevel

# Default logger format
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
# os.environ['RCUTILS_COLORIZED_OUTPUT'] = "[{file_name}:{line_number}:{function_name}] {message}"

# Set the logger level to INFO
node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

def set_logger_level(level: LoggerLevel):
    """
    Set the logger level for the pycram logger. Possible values are: DEBUG, INFO, WARN, ERROR, FATAL.

    :param level: The level to set the logger to.
    """
    lev = rclpy.logging.LoggingSeverity.INFO
    if level == LoggerLevel.DEBUG:
        lev = rclpy.logging.LoggingSeverity.DEBUG
    elif level == LoggerLevel.INFO:
        lev = rclpy.logging.LoggingSeverity.INFO
    elif level == LoggerLevel.WARN:
        lev = rclpy.logging.LoggingSeverity.WARN
    elif level == LoggerLevel.ERROR:
        lev = rclpy.logging.LoggingSeverity.ERROR
    elif level == LoggerLevel.FATAL:
        lev = rclpy.logging.LoggingSeverity.FATAL

    node.get_logger().set_level(lev)

def set_logger_format(logger_format: str):
    """
    Set the logger format for the pycram logger. The format can contain the following placeholders:
    - {severity}
    - {name}
    - {message}
    - {function_name}
    - {file_name}
    - {line_number}
    The format is set to the environment variable RCUTILS_CONSOLE_OUTPUT_FORMAT. Therefore, the format is not only
    applied to the pycram logger but to all loggers in the environment.

    :param logger_format: The format to set the logger to.
    """
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = logger_format

def logwarn(message: str):
    node.get_logger().warning(message)

def loginfo(message: str):
    node.get_logger().info(message)

def logerr(message: str):
    node.get_logger().error(message)

def logdebug(message: str):
    node.get_logger().debug(message)

def logwarn_once(message: str):
    node.get_logger().warning(message, once=True)

def loginfo_once(message: str):
    node.get_logger().info(message, once=True)

def logerr_once(message: str):
    node.get_logger().error(message, once=True)

def logdebug_once(message: str):
    node.get_logger().debug(message, once=True)