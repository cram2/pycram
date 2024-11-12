from .default_process_modules import DefaultManager


class FetchManager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "fetch"
