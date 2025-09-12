from dataclasses import dataclass

from semantic_world.world_description.world_entity import Body

from .base import BaseMotion
from ...datastructures.enums import Arms
from ...process_module import ProcessModuleManager


@dataclass
class OpeningMotion(BaseMotion):
    """
    Designator for opening container
    """

    object_part: Body
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager(self.world)
        return pm_manager.open().execute(self)


@dataclass
class ClosingMotion(BaseMotion):
    """
    Designator for closing a container
    """

    object_part: Body
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager(self.world)
        return pm_manager.close().execute(self)
