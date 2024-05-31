# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from anytree import NodeMixin
from typing_extensions import Iterable, Callable

from ..designator import DesignatorDescription


class DecisionTreeNode(NodeMixin):

    def __init__(self, parent: NodeMixin = None, children: Iterable[NodeMixin] = None):
        self.parent = parent
        self.true_path = None
        self.false_path = None
        if children:
            self.children = children

    def __add__(self, other: DecisionTreeNode) -> DecisionTreeNode:
        if isinstance(self, Decision):
            self._add_true(other)
            return self
        else:
            raise TypeError("Only Decision nodes can have children")

    def __sub__(self, other: DecisionTreeNode) -> DecisionTreeNode:
        if isinstance(self, Decision):
            self._add_false(other)
            return self
        else:
            raise TypeError("Only Decision nodes can have children")

    def perform(self, designator: DesignatorDescription):
        """
        To be implemented by subclasses
        """
        raise NotImplementedError


class Decision(DecisionTreeNode):

    def __init__(self, condition: Callable):
        super().__init__(parent=None, children=None)
        self.condition = condition

    def perform(self, designator: DesignatorDescription) -> DesignatorDescription:
        if self.condition():
            if self.true_path:
                return self.true_path.perform(designator)
            else:
                raise ValueError(f"No true path defined for decision node: {self}")
        else:
            if self.false_path:
                return self.false_path.perform(designator)
            else:
                raise ValueError(f"No false path defined for decision node: {self}")

    def _add_true(self, other):
        if not self.true_path:
            self.true_path = other
            other.parent = self
        else:
            self.true_path._add_true(other)

    def _add_false(self, other):
        if not self.false_path:
            self.false_path = other
            other.parent = self
        else:
            self.false_path._add_false(other)

    def __repr__(self):
        if self.parent:
            return f"{'+' if self.parent.true_path is self else '-'}{self.__class__.__name__}({self.condition})"
        return f"{self.__class__.__name__}({self.condition})"


class Query(DecisionTreeNode):

    def __init__(self, query: Callable):
        super().__init__(parent=None, children=None)
        self.query = query

    def perform(self, designator) -> DesignatorDescription:
        return self.query(designator)

    def __repr__(self):
        if self.parent:
            return f"{'+' if self.parent.true_path is self else '-'}{self.__class__.__name__}({self.query})"
        return f"{self.__class__.__name__}({self.query})"
