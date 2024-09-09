from .donbot_process_modules import DonbotManager
from .pr2_process_modules import Pr2Manager
from .justin_process_modules import JustinManager
from .icub_process_modules import ICubManager
#from .boxy_process_modules import BoxyManager
#from .stretch_process_modules import StretchManager
#from .hsrb_process_modules import HSRBManager
from .default_process_modules import DefaultManager
from .tiago_process_modules import TiagoManager

Pr2Manager()
JustinManager()
ICubManager()
TiagoManager()
#BoxyManager()
DonbotManager()
#StretchManager()
#HSRBManager()
DefaultManager()
