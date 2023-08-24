from .default_process_modules import DefaultManager
from .pr2_process_modules import Pr2Manager
from .boxy_process_modules import BoxyManager
from .donbot_process_modules import DonbotManager
from .hsr_process_modules import HSRManager

DefaultManager()
Pr2Manager()
BoxyManager()
DonbotManager()
HSRManager()
