# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from anytree import NodeMixin
from typing_extensions import Iterable, Callable

from ..designator import DesignatorDescription


class DecisionTreeNode(NodeMixin):
    """
    Base class for nodes in the Decision Tree
    """

    def __init__(self, parent: NodeMixin = None, children: Iterable[NodeMixin] = None):
        """
        A node in the decision tree needs a parent and can have children.
        Children can be added using the + and - operators.

        :param parent: An instance of type DecisionTreeNode which is the parent of this node
        :param children: A list of DecisionTreeNode instances which are the children of this node
        """
        self.parent = parent
        self.true_path = None
        self.false_path = None
        if children:
            self.children = children

    def __add__(self, other: DecisionTreeNode) -> DecisionTreeNode:
        """
        Overloads the "+" operator to add a child to the true path of a decision node. This child is executed if the
        condition of the decision node is true.

        :param other: DecisionTreeNode instance which should be executed if the condition of the decision node is true
        :return: This DecisionTreeNode instance
        """
        if isinstance(self, Decision):
            self._add_true(other)
            return self
        else:
            raise TypeError("Only Decision nodes can have children")

    def __sub__(self, other: DecisionTreeNode) -> DecisionTreeNode:
        """
        Overloads the "-" operator to add a child to the false path of a decision node. This child is executed if the
        condition of the decision node is false.

        :param other: DecisionTreeNode instance which should be executed if the condition of the decision node is false
        :return: This DecisionTreeNode instance
        """
        if isinstance(self, Decision):
            self._add_false(other)
            return self
        else:
            raise TypeError("Only Decision nodes can have children")

    def perform(self, designator: DesignatorDescription):
        """
        To be implemented by subclasses. Defines the behavior of the node when it is executed.
        """
        raise NotImplementedError


class Decision(DecisionTreeNode):
    """
    A decision node in the decision tree. It has a condition which is evaluated to determine the path to take.
    """

    def __init__(self, condition: Callable):
        super().__init__(parent=None, children=None)
        self.condition = condition

    def perform(self, designator: DesignatorDescription) -> DesignatorDescription:
        """
        Calls the condition function and evaluates the result to determine the path to take. If the condition is true
        the true_path is executed, otherwise the false_path is executed.

        :param designator: Designator for which the condition should be evaluated
        :return: The result of the path that was executed, which should be the result of a Query
        """
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
        """
        Adds a child to the true path of this decision node. If the true path is already defined, the child is added to
        the last node along the true path in this tree.

        :param other: DecisionTreeNode instance which should be added as the child of this node.
        """
        if not self.true_path:
            self.true_path = other
            other.parent = self
        else:
            self.true_path._add_true(other)

    def _add_false(self, other):
        """
        Adds a child to the false path of this decision node. If the false path is already defined, the child is added to
        the last node along the false path in this tree.

        :param other: DecisionTreeNode instance which should be added as the child of this node.
        """
        if not self.false_path:
            self.false_path = other
            other.parent = self
        else:
            self.false_path._add_false(other)

    def __repr__(self) -> str:
        """
        Returns a string representation of the decision node. If the node has a parent, the string representation includes
        if this node the true or false child of the parent.

        :return: A string representation of the decision node
        """
        if self.parent:
            return f"{'+' if self.parent.true_path is self else '-'}{self.__class__.__name__}({self.condition})"
        return f"{self.__class__.__name__}({self.condition})"


class Query(DecisionTreeNode):
    """
    A query node in the decision tree. It has a query function which is executed when the node is evaluated.
    This node should function as a leaf node in the decision tree.
    """

    def __init__(self, query: Callable):
        super().__init__(parent=None, children=None)
        self.query = query

    def perform(self, designator) -> DesignatorDescription:
        """
        Calls the query function of this node and returns the result.

        :param designator: The designator for which the query is used
        :return: A designator with the result of the query as a parameter
        """
        return self.query(designator)

    def __repr__(self):
        """
        Returns a string representation of the query node. If the node has a parent, the string representation includes
        if this node the true or false child of the parent.

        :return: A string representation of the query node
        """
        if self.parent:
            return f"{'+' if self.parent.true_path is self else '-'}{self.__class__.__name__}({self.query})"
        return f"{self.__class__.__name__}({self.query})"
