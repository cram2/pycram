from bullet_world_testcase import BulletWorldTestCase
from pycram.ros.logging import set_logger_level, logwarn, logerr
from pycram.datastructures.enums import LoggerLevel


class TestLogging(BulletWorldTestCase):
    """Testcase for the pycram logging functions."""

    def test_set_logger_level(self):
        logwarn("This is a warning, it should not be printed")
        logerr("This is an error, it should be printed")
        set_logger_level(LoggerLevel.ERROR)
        logwarn("This is a warning, it should not be printed")
        logerr("This is an error, it should be printed")
