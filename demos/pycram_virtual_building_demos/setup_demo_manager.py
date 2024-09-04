from ipywidgets import Output

from setup_utils import display_loading_gif


def start_demo(func):
    global output
    output = Output()
    display_loading_gif()
    func()
