import threading
import pycram_bullet as p

from pycram.failures import PlanFailure
from pycram.process_module import simulated_robot
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.enums import WorldMode, ApproachDirection, VerticalAlignment
from pycram.robot_plans import *
world = BulletWorld(WorldMode.GUI)

pr2 = Object("pr2", Robot, "pr2.urdf", pose=PoseStamped.from_list([1, 2, 0]))
apartment = Object("apartment", Apartment, "apartment.urdf")
milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([2.37, 2.35, 1.05]))


class ActionButtons(threading.Thread):
    def __init__(self, world: BulletWorld):
        self.world = world
        self.kill_event = threading.Event()
        self.torso_button = p.addUserDebugParameter("Move the Torso Up", 1, 0, 1, physicsClientId=self.world.id)
        self.torso_down_button = p.addUserDebugParameter("Move the Torso Down", 1, 0, 1, physicsClientId=self.world.id)
        self.park_arms_button = p.addUserDebugParameter("Park Arms", 1, 0, 1, physicsClientId=self.world.id)
        self.pick_up_button = p.addUserDebugParameter("Pick Up Milk", 1, 0, 1, physicsClientId=self.world.id)

        self.move_x_slider = p.addUserDebugParameter("Move X", -10, 10, 1, physicsClientId=self.world.id)
        self.move_y_slider = p.addUserDebugParameter("Move Y", -10, 10, 2, physicsClientId=self.world.id)
        self.rotate_z_slider = p.addUserDebugParameter("Rotate around z", 0, 360, 0, physicsClientId=self.world.id)

        self.transport_table = p.addUserDebugParameter("Transport Table", 1, 0, 1, physicsClientId=self.world.id)
        self.transport_island = p.addUserDebugParameter("Transport Island", 1, 0, 1, physicsClientId=self.world.id)
        self.plot_last_transport = p.addUserDebugParameter("Plot Last Transport", 1, 0, 1, physicsClientId=self.world.id)

        self.quit_button = p.addUserDebugParameter("Quit", 1, 0, 1, physicsClientId=self.world.id)



        threading.Thread.__init__(self)
        self.start()

    def run(self):
        last_torso_button_value = 1
        last_torso_down_button_value = 1
        last_quit_button_value = 1
        last_park_arms_button_value = 1
        last_pick_up_button_value = 1

        last_move_x_value = 1
        last_move_y_value = 1
        last_rotate_z_value = 0

        last_transport_table_value = 1
        last_transport_island_value = 1
        last_plot_last_transport_value = 1
        last_transport_plan = None


        with simulated_robot:
            while not self.kill_event.is_set():
                torso_button_value = p.readUserDebugParameter(self.torso_button, physicsClientId=self.world.id)

                if torso_button_value != last_torso_button_value:
                    MoveTorsoActionDescription([TorsoState.HIGH]).perform()
                    last_torso_button_value = torso_button_value

                torso_down_button_value = p.readUserDebugParameter(self.torso_down_button, physicsClientId=self.world.id)
                if torso_down_button_value != last_torso_down_button_value:
                    MoveTorsoActionDescription([TorsoState.LOW]).perform()
                    last_torso_down_button_value = torso_down_button_value

                park_arms_button_value = p.readUserDebugParameter(self.park_arms_button, physicsClientId=self.world.id)
                if park_arms_button_value != last_park_arms_button_value:
                    ParkArmsActionDescription(Arms.BOTH).perform()
                    last_park_arms_button_value = park_arms_button_value

                pick_up_button_value = p.readUserDebugParameter(self.pick_up_button, physicsClientId=self.world.id)
                if pick_up_button_value != last_pick_up_button_value:
                    try:
                        PickUpActionDescription(BelieveObject(types=[Milk]), Arms.LEFT, GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)).perform()
                    except PlanFailure:
                        print("Pick up failed")
                    last_pick_up_button_value = pick_up_button_value

                move_x_value = p.readUserDebugParameter(self.move_x_slider, physicsClientId=self.world.id)
                rotate_z_value = p.readUserDebugParameter(self.rotate_z_slider, physicsClientId=self.world.id)
                move_y_value = p.readUserDebugParameter(self.move_y_slider, physicsClientId=self.world.id)

                if move_x_value != last_move_x_value or move_y_value != last_move_y_value or rotate_z_value != last_rotate_z_value:
                    pr2.set_pose(PoseStamped.from_list([move_x_value, move_y_value, 0], quaternion_from_euler(0, 0, rotate_z_value)))
                    last_move_x_value = move_x_value
                    last_move_y_value = move_y_value
                    last_rotate_z_value = rotate_z_value

                transport_table_value = p.readUserDebugParameter(self.transport_table, physicsClientId=self.world.id)
                if transport_table_value != last_transport_table_value:
                    try:
                        transport = TransportActionDescription(milk, [PoseStamped.from_list([4.8, 3.55, 0.8])], [Arms.LEFT])
                        transport.perform()
                    except PlanFailure:
                        print("Transport failed")
                    last_transport_table_value = transport_table_value
                    last_transport_plan = transport

                transport_island_value = p.readUserDebugParameter(self.transport_island, physicsClientId=self.world.id)
                if transport_island_value != last_transport_island_value:
                    try:
                        transport = TransportActionDescription(milk, [PoseStamped.from_list([2.37, 2.35, 1.05])], [Arms.LEFT])
                        transport.perform()
                    except PlanFailure:
                        print("Transport failed")
                    last_transport_island_value = transport_island_value
                    last_transport_plan = transport

                plot_last_transport_value = p.readUserDebugParameter(self.plot_last_transport, physicsClientId=self.world.id)
                if plot_last_transport_value != last_plot_last_transport_value:
                    if last_transport_plan is not None:
                        last_transport_plan.plot_bokeh()
                    else:
                        print("No transport plan to plot")
                    last_plot_last_transport_value = plot_last_transport_value


                if p.readUserDebugParameter(self.quit_button, physicsClientId=self.world.id) != last_quit_button_value:
                    self.world.exit()
                    self.kill_event.set()

ActionButtons(world)