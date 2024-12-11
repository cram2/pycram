from IPython.core.display_functions import display
from ipywidgets import widgets

from demos.pycram_hsrb_follow.variable_handler import VariableHandler
from pycram.designators.action_designator import *
from pycram.process_module import simulated_robot
from pycram.world_concepts.world_object import Object
from pycrap import Robot, Milk, Apartment


def follow_binder_demo(robot_name = None, apartment_name = None):
    """
    Starts the "follow" demo in a binder conform version.
    Original demo from Jule Schulz relies on Keyboard input which cannot be natively used in a binder environment.
    """

    output = widgets.Output()

    vh = VariableHandler()

    # Objects
    if robot_name is None:
        robot = Object('hsrb', Robot, f"hsrb.urdf", pose=Pose([1, 2, 0]))
    else:
        robot_desig = BelieveObject(names=[robot_name])
        robot = robot_desig.resolve().world_object

    if apartment_name is None:
        apartment = Object('apartment', Apartment, f"apartment-small.urdf")
    else:
        apartment_desig = BelieveObject(names=[apartment_name])
        apartment = apartment_desig.resolve().world_object

    human = Object("human", Milk, "human.stl", pose=Pose([2, 0, 0], vh.ori_s))

    vh.set_robot(robot)
    vh.set_output(output)
    vh.set_human(human)

    def move_up(_):
        vh.move_forward()
    def move_down(_):
        vh.move_back()
    def move_left(_):
        vh.move_left()
    def move_right(_):
        vh.move_right()
    def go_back(_):
        vh.go_back()
    def reset_demo(_):
        vh.reset_demo()


    # Place robot behind human
    with simulated_robot:
        pose = Pose([1, 0, 0], vh.ori_a)
        NavigateAction([pose]).resolve().perform()

    up_button = widgets.Button(description="Forward", button_style="success")
    down_button = widgets.Button(description="Back", button_style="success")
    left_button = widgets.Button(description="Left", button_style="success")
    right_button = widgets.Button(description="Right", button_style="success")
    back_button = widgets.Button(description="Go back", button_style="warning")
    reset_button = widgets.Button(description="Reset", button_style="danger")

    # Attach event handlers
    up_button.on_click(move_up)
    down_button.on_click(move_down)
    left_button.on_click(move_left)
    right_button.on_click(move_right)
    back_button.on_click(go_back)
    reset_button.on_click(reset_demo)

    # Arrange buttons in a "WASD" layout with offset
    buttons = widgets.VBox([
        widgets.HBox([widgets.Label(" ")] * 38 + [up_button] + [widgets.Label(" ")]),  # W (Up) shifted right
        widgets.HBox([left_button, down_button, right_button]),  # A (Left), S (Down), D (Right)
        widgets.HBox([back_button] + [widgets.Label(" ")] * 38 + [reset_button])  # Reset button in the center
    ])

    # Display everything
    display(buttons, output)

    # Demo is now ready using the buttons in jupyter notebook file
