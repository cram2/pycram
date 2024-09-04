from ipywidgets import Output

from setup_utils import display_loading_gif_with_text, update_text


def start_demo(func):
    global output
    output = Output()
    text_widget = display_loading_gif_with_text()
    func()
    update_text(text_widget, 'Almost done...')
