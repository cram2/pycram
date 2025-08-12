from dataclasses import dataclass

from .base import BaseMotion
from ...datastructures.enums import Arms
from ...description import ObjectDescription
from ...process_module import ProcessModuleManager


@dataclass
class OpeningMotion(BaseMotion):
    """
    Designator for opening container
    """

    object_part: ObjectDescription.Link
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        return pm_manager.open().execute(self)


@dataclass
class ClosingMotion(BaseMotion):
    """
    Designator for closing a container
    """

    object_part: ObjectDescription.Link
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        return pm_manager.close().execute(self)
