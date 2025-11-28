from .default_process_modules import *


class ArmarManager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "Armar6"


ArmarManager()
