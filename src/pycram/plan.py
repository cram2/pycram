from __future__ import annotations

import enum
import inspect
import random
from dataclasses import field, dataclass
from datetime import datetime


import networkx as nx

from typing_extensions import Optional, Callable, Any, Dict, List, Self, Iterable, TYPE_CHECKING, Type, Tuple, Iterator

from .datastructures.enums import TaskStatus
from pycrap.ontologies import Action 
from .failures import PlanFailure
from .external_interfaces import giskard
from .ros import loginfo

if TYPE_CHECKING:
    from .designator import BaseMotion, ActionDescription


class Plan(nx.DiGraph):
    """
    Represents a plan structure, typically a tree, which can be changed at any point in time. Performing the plan will
    traverse the plan structure in depth first order and perform each PlanNode
    """
    current_plan: Plan = None

    on_start_callback: Optional[Type[Action], Callable] = None
    on_end_callback: Optional[Type[Action], Callable] = None

    def __init__(self, root: PlanNode):
        super().__init__()
        self.root: PlanNode = root
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
        previous_plan = Plan.current_plan
        Plan.current_plan = self
        result = self.root.perform()
        Plan.current_plan = previous_plan
        return result

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
    def parent(self) -> PlanNode:
        return list(self.plan.predecessors(self))[0]

    @property
    def children(self) -> List[PlanNode]:
        return list(self.plan.successors(self))

    @property
    def recursive_children(self) -> List[PlanNode]:
        return list(nx.descendants(self.plan, self))

    @property
    def subtree(self):
        return nx.subgraph(self.plan, self.recursive_children + [self])

    @property
    def all_parents(self):
        return list(nx.ancestors(self.plan, self))

    def flattened_parameters(self):
        pass

    def __hash__(self):
        return id(self)

    def perform(self, *args, **kwargs):
        pass

    def interrupt(self):
        self.status = TaskStatus.INTERRUPTED
        loginfo(f"Interrupted node: {str(self)}")
        if giskard.giskard_wrapper:
            giskard.giskard_wrapper.interrupt()

@dataclass
class DesignatorNode(PlanNode):
    designator_ref: Any = None
    """
    Reference to the Designator in this node
    """

    action: Optional[Any] = None
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
        return f"<{self.designator_ref.performable.__name__}_{id(self) % 100}>"

    def flattened_parameters(self):
        return self.designator_ref.performable.flattened_parameters()


@dataclass
class ActionNode(DesignatorNode):
    action_iter: Iterator[ActionDescription] = None
    def __hash__(self):
        return id(self)

    def perform(self, *args, **kwargs):
        self.plan.current_action = self
        if not self.action_iter:
            self.action_iter = iter(self.designator_ref)
        resolved_action = next(self.action_iter)
        resolved_action_node = ResolvedActionNode(designator_ref=resolved_action, action=resolved_action.__class__, )
        self.plan.add_edge(self, resolved_action_node)

        self.start_time = datetime.now()
        result = resolved_action_node.perform()
        self.end_time = datetime.now()
        return result

    def __repr__(self, *args, **kwargs):
        return f"<{self.designator_ref.performable.__name__}_{id(self) % 100}>"

@dataclass
class ResolvedActionNode(DesignatorNode):
    def __hash__(self):
        return id(self)

    def perform(self, *args, **kwargs):
        self.plan.current_node = self
        self.start_time = datetime.now()
        result = self.designator_ref.perform()
        self.end_time = datetime.now()
        return result

    def __repr__(self, *args, **kwargs):
        return f"<{self.designator_ref.__class__.__name__}_{id(self) % 100}>"

@dataclass
class MotionNode(DesignatorNode):
    designator_ref: BaseMotion = None
    def __hash__(self):
        return id(self)

    def perform(self, *args, **kwargs):
        all_parents_status = [parent.status for parent in self.all_parents]
        if TaskStatus.INTERRUPTED in all_parents_status:
            return
        self.start_time = datetime.now()
        result = self.designator_ref.perform()
        self.end_time = datetime.now()
        return result

    def __repr__(self, *args, **kwargs):
        return f"<{self.designator_ref.__class__.__name__}_{id(self) % 100}>"

def with_tree(func: Callable) -> Callable:
    def handle_tree(*args, **kwargs):
        return func(*args, **kwargs)
    return handle_tree


def with_plan(func: Callable) -> Callable:
    def wrapper(*args, **kwargs) -> Plan:
        designator = func(*args, **kwargs)
        if designator.__class__.__name__ ==  "PartialDesignator":
            node = ActionNode(designator_ref=designator, action=designator.performable, kwargs=designator.kwargs)
        else:
            kwargs = dict(inspect.signature(func).bind(*args, **kwargs).arguments)
            node = MotionNode(designator_ref=designator, action=designator.__class__, kwargs=kwargs)
        plan = Plan(root=node)
        if Plan.current_plan:
            Plan.current_plan.mount(plan, Plan.current_plan.current_node)
        return plan

    return wrapper

