from __future__ import annotations

import enum
import inspect
from dataclasses import field, dataclass
from datetime import datetime


import networkx as nx

from typing_extensions import Optional, Callable, Any, Dict, List, Self, Iterable, TYPE_CHECKING

from .datastructures.enums import TaskStatus
from pycrap.ontologies import Action
from .failures import PlanFailure


if TYPE_CHECKING:
    from .datastructures.partial_designator import PartialDesignator


class Plan(nx.DiGraph):
    """
    Represents a plan structure, typically a tree, which can be changed at any point in time. Performing the plan will
    traverse the plan structure in depth first order and perform each PlanNode
    """
    current_plan: Plan = None

    def __init__(self, root: PlanNode):
        super().__init__()
        self.root = root
        self.add_node(self.root)
        self.current_node: PlanNode = self.root

    def mount(self, other: Plan, mount_node: PlanNode = None):
        mount_node = mount_node or self.root
        self.add_nodes_from(other.nodes)
        self.add_edges_from(other.edges)
        self.add_edge(mount_node, other.root)
        for node in self.nodes:
            node.plan = self

    def merge_nodes(self, node1: PlanNode, node2: PlanNode):
        for node in node2.children:
            self.add_edge(node1, node)
        self.remove_node(node2)
        
    def add_node(self, node_for_adding: PlanNode, **attr):
        super().add_node(node_for_adding, **attr)
        node_for_adding.plan = self

    def add_edge(self, u_of_edge, v_of_edge, **attr):
        super().add_edge(u_of_edge, v_of_edge, **attr)
        u_of_edge.plan = self
        v_of_edge.plan = self

    def add_edges_from(self, ebunch_to_add: Iterable[Tuple[PlanNode, PlanNode]], **attr):
        super().add_edges_from(ebunch_to_add, **attr)
        for u_edge, v_edge in ebunch_to_add:
            u_edge.plan = self
            v_edge.plan = self

    def add_nodes_from(self, nodes_for_adding: Iterable[PlanNode], **attr):
        super().add_nodes_from(nodes_for_adding, **attr)
        for node in nodes_for_adding:
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

    def resolve(self):
        if isinstance(self.root, DesignatorNode):
            return self.root.designator_ref.resolve()

    def flattened_parameters(self):
        result = {}
        for node in self.nodes:
            if isinstance(node, DesignatorNode):
                result.update(node.flattened_parameters())
        return result

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
        return list(self.plan.predecessors(self))[0]

    @property
    def children(self):
        return list(self.plan.successors(self))

    def __hash__(self):
        return id(self)

    def perform(self, *args, **kwargs):
        pass

@dataclass
class DesignatorNode(PlanNode):
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


@dataclass
class ActionNode(DesignatorNode):
    def __hash__(self):
        return id(self)

@dataclass
class MotionNode(DesignatorNode):
    def __hash__(self):
        return id(self)

def with_tree(func: Callable) -> Callable:
    pass


def with_plan(func: Callable) -> Callable:
    def wrapper(*args, **kwargs) -> Plan:
        designator = func(*args, **kwargs)
        if designator.__class__.__name__ ==  "PartialDesignator":
            node = ActionNode(designator_ref=designator, action="Action", kwargs=designator.kwargs)
        else:
            kwargs = dict(inspect.signature(func).bind(*args, **kwargs).arguments)
            node = MotionNode(designator_ref=designator, action="Action", kwargs=kwargs)
        plan = Plan(root=node)
        if Plan.current_plan:
            Plan.current_plan.mount(plan, Plan.current_plan.current_node)
        return plan

    return wrapper

