import sys
import unittest

from pycram.testing import BulletWorldTestCase
from pycram.ros import set_logger_level, logwarn, logerr, logdebug
from pycram.datastructures.enums import LoggerLevel


class TestLogging(BulletWorldTestCase):
    """Testcase for the pycram logging functions."""

    @unittest.skipIf(sys.version_info.minor > 9, "ROS logging does not work in higher python versions")
    def test_set_logger_level(self):
        set_logger_level(LoggerLevel.DEBUG)
        logdebug("This is a debug message, it should be printed")
        logwarn("This is a warning, it should be printed")
        logerr("This is an error, it should be printed")
        set_logger_level(LoggerLevel.ERROR)
        logdebug("This is a debug message, it should not be printed")
        logwarn("This is a warning, it should not be printed")
        logerr("This is an error, it should be printed")
