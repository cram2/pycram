from .default_process_modules import DefaultManager

try:
    from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2
except ImportError:
    pass


class Pr2Manager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "pr2"
