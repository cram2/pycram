from dataclasses import dataclass

from semantic_digital_twin.robots.abstract_robot import AbstractRobot

from ..motion_statecharts import AlternativeMotionMapping

class HSRBTCPMotionMapping(AlternativeMotionMapping):

    @property
    def motion_chart(self):
        return self.robot_view.get_motion_chart("hsrb_tcp_motion")