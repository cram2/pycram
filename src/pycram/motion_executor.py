from dataclasses import dataclass, field
from typing import List

from giskardpy.executor import Executor
from giskardpy.motion_statechart.goals.templates import Sequence
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.graph_node import Task
from giskardpy.qp.qp_controller_config import QPControllerConfig
from semantic_digital_twin.world import World

from pycram.datastructures.enums import ExecutionType
from pycram.process_module import ProcessModuleManager


@dataclass
class MotionExecutor:
    motions: List[Task]
    """
    The motions to execute
    """

    world: World

    motion_state_chart: MotionStatechart = field(init=False)


    def construct_msc(self):
        self.motion_state_chart = MotionStatechart()
        sequence_node = Sequence(nodes=self.motions)
        self.motion_state_chart.add_node(sequence_node)

        self.motion_state_chart.add_node(EndMotion.when_true(sequence_node))


    def execute(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
           self._execute_for_simulation()
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            self._execute_for_real()

    def _execute_for_simulation(self):
        executor = Executor(self.world, controller_config=QPControllerConfig.create_default_with_50hz())
        executor.compile(self.motion_state_chart)
        executor.tick_until_end(timeout=2000)

    def _execute_for_real(self):
        pass
