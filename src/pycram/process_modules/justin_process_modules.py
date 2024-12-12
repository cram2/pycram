from .default_process_modules import *


class JustinManager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "rollin_justin"
