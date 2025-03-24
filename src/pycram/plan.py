from __future__ import annotations

from dataclasses import field, dataclass
from datetime import datetime

import networkx as nx

from typing_extensions import Optional, Callable, Any, Dict, List

from .datastructures.enums import TaskStatus
from pycrap.ontologies import Action
from .datastructures.partial_designator import PartialDesignator
from .failures import PlanFailure
from .helper import Singleton


class Plan(metaclass=Singleton):
    """
    Represents a plan structure, typically a tree, which can be changed at any point in time. Performing the plan will
    traverse the plan structure in depth first order and perform each PlanNode
    """

    def __init__(self):
        self.graph: nx.DiGraph = nx.DiGraph()
        self.root: PlanNode = PlanNode(None, None, TaskStatus.CREATED, None)
        self.graph.add_node(self.root)
        self.current_node: PlanNode = self.root

    def insert_node(self, node: PlanNode, path: str = None):
        if path:
            pass
        else:
            self.graph.add_edge(self.root, node)

    def insert_below(self, node: PlanNode):
        pass

    def insert_after(self, node: PlanNode):
        pass

    def find_node(self, designator) -> str:
        pass

    def perform(self):
        pass


@dataclass
class PlanNode:
    designator_ref: PartialDesignator
    """
    Reference to the Designator in this node
    """

    action: Optional[Action]
    """
    The action and that is performed or None if nothing was performed
    """

    status: TaskStatus
    """
    The status of the node from the TaskStatus enum.
    """

    kwargs: Dict[str, Any]
    """
    kwargs of the action in this node
    """

    start_time: Optional[datetime] = field(default_factory=datetime.now)
    """
    The starting time of the function, optional
    """

    end_time: Optional[datetime] = None
    """
    The ending time of the function, optional
    """

    reason: Optional[PlanFailure] = None
    """
    The reason of failure if the action failed.
    """

    parent: Optional[PlanNode] = None

    children: Optional[List[PlanNode]] = None


def with_tree(func: Callable) -> Callable:
    pass


def with_plan(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        action = func(*args, **kwargs)
        # TODO Maybe use bind_partial instead
        node = PlanNode(action, "Action", TaskStatus.CREATED, action.kwargs)
        Plan().insert_node(node)
        return action

    return wrapper
