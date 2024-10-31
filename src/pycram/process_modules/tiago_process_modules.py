from .default_process_modules import *


class TiagoManager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "tiago_dual"
