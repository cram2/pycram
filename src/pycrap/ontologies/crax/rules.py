from __future__ import annotations

from abc import ABC, abstractmethod

from typing_extensions import Dict, Type

from .object_properties import *


class CRAXRule(ABC):

    @property
    def name(self):
        return self.__class__.__name__

    @property
    @abstractmethod
    def rule(self):
        pass


class HierarchicalContainment(CRAXRule):

    @property
    def rule(self):
        return f"""
        {is_part_of}(?part, ?parent), {contains_object}(?part, ?object)
        -> {contains_object}(?parent, ?object)
        """
