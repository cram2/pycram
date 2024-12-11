from .default_process_modules import DefaultManager


class ICubManager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "iCub3"
