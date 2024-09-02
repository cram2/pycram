import ipywidgets as widgets
from IPython.core.display_functions import display
from ipywidgets import Output, Button, HBox

robot_one = [('Select', None), ('PR2', "pr2"), ('Tiago', "tiago_dual"), ('Armar6', "Armar6")]
robot_two = [('Select', None), ('PR2', "pr2"), ('Tiago', "tiago_dual"), ('Armar6', "Armar6")]
demos = [('Select', None), ('PR2', "pr2")]

first_selected_robot = None
second_selected_robot = None
selected_environment = None


def setup_task_object_widgets():
    robot_one_dropdown = widgets.Dropdown(options=robot_one, description='First Robot:')
    robot_two_dropdown = widgets.Dropdown(options=robot_two, description='Second Robot:')
    environment_dropdown = widgets.Dropdown(options=demos, description='Demo:')

    robot_one_dropdown.observe(lambda change: update_globals(robot_one=change['new']), names='value')
    robot_two_dropdown.observe(lambda change: update_globals(robot_two=change['new']), names='value')
    environment_dropdown.observe(lambda change: update_globals(environment=change['new']), names='value')

    display(HBox([robot_one_dropdown, robot_two_dropdown, environment_dropdown]))


def update_globals(robot_one=None, robot_two=None, environment=None):
    global first_selected_robot, second_selected_robot, selected_environment
    if robot_one is not None:
        first_selected_robot = robot_one
    if robot_two is not None:
        second_selected_robot = robot_two
    if environment is not None:
        selected_environment = environment


def robot_execute(func):
    global first_selected_robot, second_selected_robot, selected_environment
    with output:
        output.clear_output()
        func(first_selected_robot, second_selected_robot, selected_environment)


def start_demo(func):
    global output
    output = Output()
    #setup_task_object_widgets()
    execute_button = Button(description="Execute Task")
    # Use a lambda function to defer the call to `robot_execute`
    # In this lambda function, lambda x: robot_execute(func),
    # x represents the button click event (which we don't use here),
    # and robot_execute(func) is the function call you want to happen when the button is clicked.
    # This way, robot_execute will only be executed when the button is clicked, not when start_demo is called.
    execute_button.on_click(lambda x: robot_execute(func))
    display(execute_button, output)
