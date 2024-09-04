from ipywidgets import Output

from demos.pycram_virtual_building_demos.setup.utils import display_loading_gif


def start_demo(func):
    global output
    output = Output()
    display_loading_gif()
    func()
