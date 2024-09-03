import os

from IPython.core.display_functions import display
from ipywidgets import HTML


def display_loading_gif():
    import base64
    setup_path = os.path.dirname(os.path.realpath(__file__))
    gif_path = setup_path + '/imgs/loading.gif'
    b64 = base64.b64encode(open(gif_path, 'rb').read()).decode('ascii')
    display(HTML(f'<img src="data:image/gif;base64,{b64}" />'))
