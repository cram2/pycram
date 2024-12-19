from .pr2_process_modules import Pr2Manager
from .boxy_process_modules import BoxyManager
from .donbot_process_modules import DonbotManager
from .hsrb_process_modules import HSRBManager
from .default_process_modules import DefaultManager
from .stretch_process_modules import StretchManager
from .robotiq_gripper_process_module import RobotiqManager
from .tiago_process_modules import TiagoManager

Pr2Manager()
BoxyManager()
DonbotManager()
HSRBManager()
DefaultManager()
StretchManager()
TiagoManager()
RobotiqManager()
