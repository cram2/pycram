from __future__ import annotations

from dataclasses import field, dataclass
from datetime import datetime


import networkx as nx


from typing_extensions import Optional, Callable, Any, Dict, List, Self

from .datastructures.enums import TaskStatus
from pycrap.ontologies import Action
from .datastructures.partial_designator import PartialDesignator
from .failures import PlanFailure


class Plan(nx.DiGraph):
    """
    Represents a plan structure, typically a tree, which can be changed at any point in time. Performing the plan will
    traverse the plan structure in depth first order and perform each PlanNode
    """
    current_plan: Plan = None

    def __init__(self):
        super().__init__()
        self.root: PlanNode = PlanNode(plan=self)
        self.add_node(self.root)
        self.current_node: PlanNode = self.root

    def mount(self, other: Plan, mount_node: PlanNode = None):
        mount_node = mount_node or self.root
        self.add_nodes_from(other.nodes)
        self.add_edges_from(other.edges)
        self.merge_nodes(mount_node, other.root)
        for node in self.nodes:
            node.plan = self

    def merge_nodes(self, node1: PlanNode, node2: PlanNode):
        for node in node2.children:
            self.add_edge(node1, node)
        self.remove_node(node2)

    def insert_node(self, node: PlanNode, path: str = None):
        if path:
            pass
        else:
            self.add_edge(self.root, node)
            node.plan = self

    def insert_below(self, insert_node: PlanNode, insert_below: PlanNode):
        self.add_edge(insert_below, insert_node)

    def insert_after(self, node: PlanNode):
        pass

    def find_node(self, designator) -> str:
        pass

    def find_plan(self, plan: Plan):
        plan_root = plan.root
        for node in self.nodes:
            if node == plan_root:
                return node

    def perform(self):
        pass

    def __enter__(self):
        self.prev_plan = Plan.current_plan
        Plan.current_plan = self

    def __exit__(self, exc_type, exc_val, exc_tb):
        Plan.current_plan = self.prev_plan


@dataclass
class PlanNode:
    status: TaskStatus = TaskStatus.CREATED
    """
    The status of the node from the TaskStatus enum.
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

    plan: Plan = None
    """
    Reference to the plan to which this node belongs
    """

    @property
    def parent(self):
        return list(filter(None, [edge[0] if edge[1] == self else None for edge in  self.plan.edges]))

    @property
    def children(self):
        return list(filter(None, [edge[1] if edge[0] == self else None for edge in  self.plan.edges]))

    def __hash__(self):
        return id(self)


@dataclass
class ActionNode(PlanNode):
    designator_ref: PartialDesignator = None
    """
    Reference to the Designator in this node
    """

    action: Optional[Action] = None
    """
    The action and that is performed or None if nothing was performed
    """

    kwargs: Dict[str, Any] = None
    """
    kwargs of the action in this node
    """

    def __hash__(self):
        return id(self)

    def __repr__(self, *args, **kwargs):
        return f"<{self.designator_ref.performable.__name__}_{id(self)}>"


def with_tree(func: Callable) -> Callable:
    pass


def with_plan(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        action = func(*args, **kwargs)
        # TODO Maybe use bind_partial instead
        node = ActionNode(designator_ref=action, action="Action", kwargs=action.kwargs)
        plan = Plan()
        plan.insert_node(node)
        if Plan.current_plan:
            Plan.current_plan.mount(plan)
        return plan

    return wrapper
