import logging
import re

import rospy

from .boxy_description import BoxyDescription
from .donbot_description import DonbotDescription
from .hsr_description import HSRDescription
from .pr2_description import PR2Description
from .ur5_description import UR5Description
from .tiago_description import TiagoDescription
from .. import utils
from ..robot_description import RobotDescription

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class InitializedRobotDescription():
    # singleton instance short named as 'i'
    i = None
    current_description_loaded = None

    def __init__(self, robot_description):
        assert issubclass(robot_description, RobotDescription) and robot_description is not RobotDescription
        if not InitializedRobotDescription.i or \
                InitializedRobotDescription.current_description_loaded is not robot_description:
            InitializedRobotDescription.current_description_loaded = robot_description
            InitializedRobotDescription.i = robot_description()
            logger.info("(robot-description) (Re)Loaded Description of robot %s.", self.i.name)


def update_robot_description(robot_name=None, from_ros=None):
    # Get robot name
    if robot_name:
        robot = robot_name
    elif from_ros:
        try:
            urdf = rospy.get_param('robot_description')
        except Exception as e:
            logger.error("(robot-description) Could not get robot name from parameter server. Try again.")
            return None
        res = re.findall(r"robot\ *name\ *=\ *\"\ *[a-zA-Z_0-9]*\ *\"", urdf)
        if len(res) == 1:
            begin = res[0].find("\"")
            end = res[0][begin + 1:].find("\"")
            robot = res[0][begin + 1:begin + 1 + end].lower()
    else:
        return None

    # Choose Description based on robot name
    if 'iai_donbot' in robot:
        description = DonbotDescription
    elif 'pr2' in robot:
        description = PR2Description
    elif 'boxy' in robot:
        description = BoxyDescription
    elif 'hsr' in robot:
        description = HSRDescription
    elif "ur5_robotiq" in robot:
        description = UR5Description
    elif "tiago_dual" in robot:
        description = TiagoDescription
    else:
        logger.error("(robot-description) The given robot name %s has no description class.", robot_name)
        return None
    return InitializedRobotDescription(description)


with utils.suppress_stdout_stderr():
    update_robot_description(from_ros=True)  # "ur5_robotiq")#  # todo: put in ros init

robot_description = InitializedRobotDescription.i
