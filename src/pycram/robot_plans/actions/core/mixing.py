from pycram.robot_plans.actions.base import ActionDescription
from __future__ import annotations

import math
from dataclasses import dataclass

from pycram.has_parameters import has_parameters
from pycram.plan import with_plan

from pycram.datastructures.partial_designator import PartialDesignator

from typing_extensions import Union, Optional, Iterable

from pycram.designators.motion_designator import MoveTCPMotion
from pycram.local_transformer import LocalTransformer

from pycram.datastructures.enums import Arms

from pycram.datastructures.world import World

from pycram.world_concepts.world_object import Object


@has_parameters
@dataclass
class MixingAction(ActionDescription):
    object_: Object
    tool: Object
    arm: Arms
    technique: Optional[str] = None

    def plan(self) -> None:
        lt = LocalTransformer()
        obj = self.object_
        pose = lt.transform_to_object_frame(obj.pose, obj)
        height_offset = obj.size[2] + 0.05

        for t in range(20):  # 2 * steps, with steps=10
            p = pose.copy()
            r, a, h = 0.0035 * t, math.radians(30) * t, 0.001 * t
            p.pose.position.x += r * math.cos(a)
            p.pose.position.y += r * math.sin(a)
            p.pose.position.z += h

            spiral = lt.transform_pose(p, "map")
            spiral.pose.position.z += height_offset
            World.current_world.add_vis_axis(spiral)
            MoveTCPMotion(spiral, self.arm).perform()

        World.current_world.remove_vis_axis()

    @classmethod
    @with_plan
    def description(cls, object_: Union[Iterable[Object], Object], tool: Union[Iterable[Object], Object],
                    arm: Optional[Union[Iterable[Arms], Arms]] = None,
                    technique: Optional[Union[Iterable[str], str]] = None):
        return PartialDesignator(cls, object_=object_, tool=tool, arm=arm, technique=technique)

MixingActionDescription = MixingAction.description