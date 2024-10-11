import rospy
import inspect
from pathlib import Path


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


def logwarn(message: str):
    rospy.logwarn(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}")


def loginfo(message: str):
    rospy.loginfo(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}")


def logerr(message: str):
    rospy.logerr(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}")


def logdebug(message: str):
    rospy.logdebug(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}")


def logwarn_once(message: str):
    rospy.logwarn_once(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}")


def loginfo_once(message: str):
    rospy.loginfo_once(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}")


def logerr_once(message: str):
    rospy.logerr_once(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}")


def logdebug_once(message: str):
    rospy.logdebug_once(f"[{_get_caller_file_name()}:{_get_caller_method_line()}:{_get_caller_method_name()}] {message}")
