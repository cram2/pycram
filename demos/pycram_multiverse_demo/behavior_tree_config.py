from abc import ABC, abstractmethod
from typing import Optional

from giskardpy.exceptions import SetupException
from giskardpy.god_map import god_map
from giskardpy.model.ros_msg_visualization import VisualizationMode
from giskardpy.tree.behaviors.tf_publisher import TfPublishingModes
from giskardpy.tree.branches.giskard_bt import GiskardBT
from giskardpy.tree.control_modes import ControlModes
from giskardpy.utils.utils import is_running_in_pytest


class BehaviorTreeConfig(ABC):

    def __init__(self, mode: ControlModes, control_loop_max_hz: float = 50, simulation_max_hz: Optional[float] = None):
        """

        :param mode: Defines the default setup of the behavior tree.
        :param control_loop_max_hz: if mode == ControlModes.standalone: limits the simulation speed
                       if mode == ControlModes.open_loop: limits the control loop of the base tracker
                       if mode == ControlModes.close_loop: limits the control loop
        """
        self._control_mode = mode
        self.control_loop_max_hz = control_loop_max_hz
        self.simulation_max_hz = simulation_max_hz

    @abstractmethod
    def setup(self):
        """
        Implement this method to configure the behavior tree using it's self. methods.
        """

    @property
    def tree(self) -> GiskardBT:
        return god_map.tree

    def _create_behavior_tree(self):
        god_map.tree = GiskardBT(control_mode=self._control_mode)

    def set_defaults(self):
        pass

    def set_tree_tick_rate(self, rate: float = 0.05):
        """
        How often the tree ticks per second.
        :param rate: in /s
        """
        self.tree_tick_rate = rate

    def add_visualization_marker_publisher(self,
                                           mode: VisualizationMode,
                                           add_to_sync: Optional[bool] = None,
                                           add_to_control_loop: Optional[bool] = None):
        """

        :param add_to_sync: Markers are published while waiting for a goal.
        :param add_to_control_loop: Markers are published during the closed loop control sequence, this is slow.
        :param use_decomposed_meshes: True: publish decomposed meshes used for collision avoidance, these likely only
                                            available on the machine where Giskard is running.
                                      False: use meshes defined in urdf.
        """
        if add_to_sync:
            self.tree.wait_for_goal.publish_state.add_visualization_marker_behavior(mode)
        if add_to_control_loop:
            self.tree.control_loop_branch.publish_state.add_visualization_marker_behavior(mode)

    def add_qp_data_publisher(self, publish_lb: bool = False, publish_ub: bool = False,
                              publish_lbA: bool = False, publish_ubA: bool = False,
                              publish_bE: bool = False, publish_Ax: bool = False,
                              publish_Ex: bool = False, publish_xdot: bool = False,
                              publish_weights: bool = False, publish_g: bool = False,
                              publish_debug: bool = False, add_to_base: bool = False):
        """
        QP data is streamed and can be visualized in e.g. plotjuggler. Useful for debugging.
        """
        self.add_evaluate_debug_expressions()
        if god_map.is_open_loop():
            self.tree.execute_traj.base_closed_loop.publish_state.add_qp_data_publisher(
                publish_lb=publish_lb,
                publish_ub=publish_ub,
                publish_lbA=publish_lbA,
                publish_ubA=publish_ubA,
                publish_bE=publish_bE,
                publish_Ax=publish_Ax,
                publish_Ex=publish_Ex,
                publish_xdot=publish_xdot,
                publish_weights=publish_weights,
                publish_g=publish_g,
                publish_debug=publish_debug)
        else:
            self.tree.control_loop_branch.publish_state.add_qp_data_publisher(
                publish_lb=publish_lb,
                publish_ub=publish_ub,
                publish_lbA=publish_lbA,
                publish_ubA=publish_ubA,
                publish_bE=publish_bE,
                publish_Ax=publish_Ax,
                publish_Ex=publish_Ex,
                publish_xdot=publish_xdot,
                publish_weights=publish_weights,
                publish_g=publish_g,
                publish_debug=publish_debug)

    def add_trajectory_plotter(self, normalize_position: bool = False, wait: bool = False):
        """
        Plots the generated trajectories.
        :param normalize_position: Positions are centered around zero.
        :param wait: True: Behavior tree waits for this plotter to finish.
                     False: Plot is generated in a separate thread to not slow down Giskard.
        """
        self.tree.cleanup_control_loop.add_plot_trajectory(normalize_position, wait)

    def add_trajectory_visualizer(self):
        self.tree.cleanup_control_loop.add_visualize_trajectory()

    def add_debug_trajectory_visualizer(self):
        self.tree.cleanup_control_loop.add_debug_visualize_trajectory()

    def add_debug_trajectory_plotter(self, normalize_position: bool = False, wait: bool = False):
        """
        Plots debug expressions defined in goals.
        """
        self.add_evaluate_debug_expressions()
        self.tree.cleanup_control_loop.add_plot_debug_trajectory(normalize_position=normalize_position,
                                                                 wait=wait)

    def add_gantt_chart_plotter(self):
        self.add_evaluate_debug_expressions()
        self.tree.cleanup_control_loop.add_plot_gantt_chart()

    def add_goal_graph_plotter(self):
        self.add_evaluate_debug_expressions()
        self.tree.prepare_control_loop.add_plot_goal_graph()

    def add_debug_marker_publisher(self):
        """
        Publishes debug expressions defined in goals.
        """
        self.add_evaluate_debug_expressions()
        self.tree.control_loop_branch.publish_state.add_debug_marker_publisher()

    def add_tf_publisher(self, include_prefix: bool = True, tf_topic: str = 'tf',
                         mode: TfPublishingModes = TfPublishingModes.attached_and_world_objects):
        """
        Publishes tf for Giskard's internal state.
        """
        self.tree.wait_for_goal.publish_state.add_tf_publisher(include_prefix=include_prefix,
                                                               tf_topic=tf_topic,
                                                               mode=mode)
        if god_map.is_standalone():
            self.tree.control_loop_branch.publish_state.add_tf_publisher(include_prefix=include_prefix,
                                                                   tf_topic=tf_topic,
                                                                   mode=mode)

    def add_evaluate_debug_expressions(self):
        self.tree.prepare_control_loop.add_compile_debug_expressions()
        if god_map.is_closed_loop():
            self.tree.control_loop_branch.add_evaluate_debug_expressions(log_traj=False)
        else:
            self.tree.control_loop_branch.add_evaluate_debug_expressions(log_traj=True)
        if god_map.is_open_loop():
            god_map.tree.execute_traj.prepare_base_control.add_compile_debug_expressions()
            god_map.tree.execute_traj.base_closed_loop.add_evaluate_debug_expressions(log_traj=False)

    def add_js_publisher(self, topic_name: Optional[str] = None, include_prefix: bool = False):
        """
        Publishes joint states for Giskard's internal state.
        """
        god_map.tree.control_loop_branch.publish_state.add_joint_state_publisher(include_prefix=include_prefix,
                                                                                 topic_name=topic_name,
                                                                                 only_prismatic_and_revolute=True)
        god_map.tree.wait_for_goal.publish_state.add_joint_state_publisher(include_prefix=include_prefix,
                                                                           topic_name=topic_name,
                                                                           only_prismatic_and_revolute=True)

    def add_free_variable_publisher(self, topic_name: Optional[str] = None, include_prefix: bool = False):
        """
        Publishes joint states for Giskard's internal state.
        """
        god_map.tree.control_loop_branch.publish_state.add_joint_state_publisher(include_prefix=include_prefix,
                                                                                 topic_name=topic_name,
                                                                                 only_prismatic_and_revolute=False)
        god_map.tree.wait_for_goal.publish_state.add_joint_state_publisher(include_prefix=include_prefix,
                                                                           topic_name=topic_name,
                                                                           only_prismatic_and_revolute=False)


class StandAloneBTConfig(BehaviorTreeConfig):
    def __init__(self,
                 debug_mode: bool = False,
                 publish_js: bool = False,
                 visualization_mode: VisualizationMode = VisualizationMode.VisualsFrameLocked,
                 publish_free_variables: bool = False,
                 publish_tf: bool = True,
                 include_prefix: bool = False,
                 simulation_max_hz: Optional[float] = None):
        """
        The default behavior tree for Giskard in standalone mode. Make sure to set up the robot interface accordingly.
        :param debug_mode: enable various debugging tools.
        :param publish_js: publish current world state.
        :param publish_tf: publish all link poses in tf.
        :param simulation_max_hz: if not None, will limit the frequency of the simulation.
        :param include_prefix: whether to include the robot name prefix when publishing joint states or tf
        """
        self.include_prefix = include_prefix
        self.visualization_mode = visualization_mode
        if is_running_in_pytest():
            if god_map.is_in_github_workflow():
                publish_js = False
                publish_tf = False
                debug_mode = False
                simulation_max_hz = None
        super().__init__(ControlModes.standalone, simulation_max_hz=simulation_max_hz)
        self.debug_mode = debug_mode
        self.publish_js = publish_js
        self.publish_free_variables = publish_free_variables
        self.publish_tf = publish_tf
        if publish_js and publish_free_variables:
            raise SetupException('publish_js and publish_free_variables cannot be True at the same time.')

    def setup(self):
        self.add_visualization_marker_publisher(add_to_sync=True, add_to_control_loop=True,
                                                mode=self.visualization_mode)
        if self.publish_tf:
            self.add_tf_publisher(include_prefix=self.include_prefix, mode=TfPublishingModes.all)
        self.add_gantt_chart_plotter()
        self.add_goal_graph_plotter()
        if self.debug_mode:
            self.add_trajectory_plotter(wait=True)
            # self.add_trajectory_visualizer()
            # self.add_debug_trajectory_visualizer()
            self.add_debug_trajectory_plotter(wait=True)
            self.add_debug_marker_publisher()
        # self.add_debug_marker_publisher()
        if self.publish_js:
            self.add_js_publisher(include_prefix=self.include_prefix)
        if self.publish_free_variables:
            self.add_free_variable_publisher()


class OpenLoopBTConfig(BehaviorTreeConfig):
    def __init__(self,
                 debug_mode: bool = False,
                 control_loop_max_hz: float = 50,
                 visualization_mode: VisualizationMode = VisualizationMode.CollisionsDecomposed,
                 simulation_max_hz: Optional[float] = None):
        """
        The default behavior tree for Giskard in open-loop mode. It will first plan the trajectory in simulation mode
        and then publish it to connected joint trajectory followers. The base trajectory is tracked with a closed-loop
        controller.
        :param debug_mode:  enable various debugging tools.
        :param control_loop_max_hz: if not None, limits the frequency of the base trajectory controller.
        """
        super().__init__(ControlModes.open_loop, control_loop_max_hz=control_loop_max_hz,
                         simulation_max_hz=simulation_max_hz)
        if god_map.is_in_github_workflow():
            debug_mode = False
        self.debug_mode = debug_mode
        self.visualization_mode = visualization_mode

    def setup(self):
        self.add_visualization_marker_publisher(add_to_sync=True, add_to_control_loop=True,
                                                mode=self.visualization_mode)
        self.add_gantt_chart_plotter()
        self.add_goal_graph_plotter()
        if self.debug_mode or True:
            self.add_trajectory_plotter(wait=True)
            self.add_debug_trajectory_plotter(wait=True)
            self.add_debug_marker_publisher()
            self.add_qp_data_publisher(
                publish_debug=True,
                publish_xdot=True,
                # publish_lbA=True,
                # publish_ubA=True
            )


class ClosedLoopBTConfig(BehaviorTreeConfig):
    def __init__(self, debug_mode: bool = False,
                 control_loop_max_hz: float = 50,
                 visualization_mode: VisualizationMode = VisualizationMode.CollisionsDecomposed,
                 simulation_max_hz: Optional[float] = None):
        """
        The default configuration for Giskard in closed loop mode. Make use to set up the robot interface accordingly.
        :param debug_mode: If True, will publish debug data on topics. This will significantly slow down the control loop.
        :param control_loop_max_hz: Limits the control loop frequency. If None, it will go as fast as possible.
        """
        super().__init__(ControlModes.close_loop, control_loop_max_hz=control_loop_max_hz,
                         simulation_max_hz=simulation_max_hz)
        if god_map.is_in_github_workflow():
            debug_mode = False
        self.debug_mode = debug_mode
        self.visualization_mode = visualization_mode

    def setup(self):
        self.add_visualization_marker_publisher(add_to_sync=True, add_to_control_loop=False,
                                                mode=self.visualization_mode)
        # self.add_qp_data_publisher(publish_xdot=True, publish_lb=True, publish_ub=True)
        self.add_gantt_chart_plotter()
        self.add_goal_graph_plotter()
        if self.debug_mode:
            self.add_trajectory_plotter(wait=True)
            self.add_debug_trajectory_plotter(wait=True)
            self.add_debug_marker_publisher()
            # self.add_qp_data_publisher(
            #     publish_debug=True,
            #     publish_xdot=True,
            #     # publish_lbA=True,
            #     # publish_ubA=True
            # )
