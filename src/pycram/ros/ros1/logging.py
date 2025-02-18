from __future__ import annotations

import rospy
from rospy.logger_level_service_caller import LoggerLevelServiceCaller
import inspect
from pathlib import Path

from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from ..datastructures.enums import LoggerLevel


PYCRAM_LOGGER_NAME = "pycram"
logger_level_service_caller = LoggerLevelServiceCaller()



def _get_caller_method_name():
    """
    Get the name of the method that called the function from which this function is called. It is intended as a helper
    function for the log functions.

    :return: Name of the method that called the function from which this function is called.
    """
    return inspect.stack()[2][3]


def _get_caller_method_line():
    """
    Get the line of the method that called the function from which this function is called. It is intended as a helper
    function for the log functions.

    :return: Line number of the method that called the function from which this function is called.
    """
    return inspect.stack()[2][2]


def _get_caller_file_name():
    """
    Get the file name of the method that called the function from which this function is called. It is intended as a helper
    function for the log functions.

    :return: File name of the method that called the function from which this function is called.
    """
    path = Path(inspect.stack()[2][1])
    return path.name


def set_logger_level(level: LoggerLevel):
    """
    Set the logger level for the pycram logger.

    :param level: The level to set the logger to, possible values are: rospy.DEBUG, rospy.INFO, rospy.WARN, rospy.ERROR,
    rospy.FATAL.
    """
    logger_level_service_caller.get_loggers(rospy.get_name())
    logger_level_service_caller.send_logger_change_message(rospy.get_name(),
                                                           f"rosout.{PYCRAM_LOGGER_NAME}",
                                                           level.value)


def logwarn(message: str):
    rospy.logwarn(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}",
                  logger_name=PYCRAM_LOGGER_NAME)


def loginfo(message: str):
    rospy.loginfo(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}",
                  logger_name=PYCRAM_LOGGER_NAME)


def logerr(message: str):
    rospy.logerr(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}",
                 logger_name=PYCRAM_LOGGER_NAME)


def logdebug(message: str):
    rospy.logdebug(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}",
                   logger_name=PYCRAM_LOGGER_NAME)


def logwarn_once(message: str):
    rospy.logwarn_once(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}",
                       logger_name=PYCRAM_LOGGER_NAME)


def loginfo_once(message: str):
    rospy.loginfo_once(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}",
                       logger_name=PYCRAM_LOGGER_NAME)


def logerr_once(message: str):
    rospy.logerr_once(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}",
                      logger_name=PYCRAM_LOGGER_NAME)


def logdebug_once(message: str):
    rospy.logdebug_once(
        f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}",
        logger_name=PYCRAM_LOGGER_NAME)
