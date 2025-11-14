from abc import abstractmethod, ABC
from dataclasses import dataclass
from typing import Optional

from giskardpy.motion_statechart.tasks.task import Task
from semantic_digital_twin.robots.abstract_robot import AbstractRobot

from pycram.robot_plans import BaseMotion


@dataclass
class AlternativeMotionMapping(ABC):
    robot_view: AbstractRobot

    motion: BaseMotion

    @property
    @abstractmethod
    def motion_chart(self) -> Task:
        pass

    @staticmethod
    def check_for_alternative(robot_view: AbstractRobot, motion: BaseMotion) -> Optional[Task]:
        for alternative in AlternativeMotionMapping.__subclasses__():
            if alternative.robot_view == robot_view and alternative.motion == motion:
                return alternative.motion_chart
        return None