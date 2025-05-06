from __future__ import annotations

import enum
import inspect
import random
import time
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

    on_start_callback: Dict[Type[ActionDescription], List[Callable]] = field(default_factory=dict)
    on_end_callback: Dict[Type[ActionDescription], List[Callable]] = field(default_factory=dict)

    def __init__(self, root: PlanNode):
        super().__init__()
        self.root: PlanNode = root
        self.add_node(self.root)
        self.current_node: PlanNode = self.root

    def mount(self, other: Plan, mount_node: PlanNode = None):
        """
        Mounts another plan to this plan. The other plan will be added as a child of the mount_node.

        :param other: The plan to be mounted
        :param mount_node: A node of this plan to which the other plan will be mounted. If None, the root of this plan will be used.
        """
        mount_node = mount_node or self.root
        self.add_nodes_from(other.nodes)
        self.add_edges_from(other.edges)
        self.add_edge(mount_node, other.root)
        for node in self.nodes:
            node.plan = self

    def merge_nodes(self, node1: PlanNode, node2: PlanNode):
        """
        Merges two nodes into one. The node2 will be removed and all its children will be added to node1.

        :param node1: Node which will remain in the plan
        :param node2: Node which will be removed from the plan
        """
        for node in node2.children:
            self.add_edge(node1, node)
        self.remove_node(node2)

    def add_node(self, node_for_adding: PlanNode, **attr):
        """
        Adds a node to the plan. The node will not be connected to any other node of the plan.

        :param node_for_adding: Node to be added
        :param attr: Additional attributes to be added to the node
        """
        super().add_node(node_for_adding, **attr)
        node_for_adding.plan = self

    def add_edge(self, u_of_edge, v_of_edge, **attr):
        """
        Adds an edge to the plan. If one or both nodes are not in the plan, they will be added to the plan.

        :param u_of_edge: Origin node of the edge
        :param v_of_edge: Target node of the edge
        :param attr: Additional attributes to be added to the edge
        """
        super().add_edge(u_of_edge, v_of_edge, **attr)
        u_of_edge.plan = self
        v_of_edge.plan = self

    def add_edges_from(self, ebunch_to_add: Iterable[Tuple[PlanNode, PlanNode]], **attr):
        """
        Adds edges to the plan from an iterable of tuples. If one or both nodes are not in the plan, they will be added to the plan.

        :param ebunch_to_add: Iterable of tuples of nodes to be added
        :param attr: Additional attributes to be added to the edges
        """
        super().add_edges_from(ebunch_to_add, **attr)
        for u_edge, v_edge in ebunch_to_add:
            u_edge.plan = self
            v_edge.plan = self

    def add_nodes_from(self, nodes_for_adding: Iterable[PlanNode], **attr):
        """
        Adds nodes from an Iterable of nodes.

        :param nodes_for_adding: The iterable of nodes
        :param attr: Addotional attributes to be added
        """
        super().add_nodes_from(nodes_for_adding, **attr)
        for node in nodes_for_adding:
            node.plan = self

    def insert_below(self, insert_node: PlanNode, insert_below: PlanNode):
        """
        Inserts a node below the given node.

        :param insert_node: The node to be inserted
        :param insert_below: A node of the plan below which the given node should be added
        """
        self.add_edge(insert_below, insert_node)

    def perform(self) -> Any:
        """
        Performs the root node of this plan.

        :return: The return value of the root node
        """
        previous_plan = Plan.current_plan
        Plan.current_plan = self
        result = self.root.perform()
        Plan.current_plan = previous_plan
        return result

    def resolve(self):
        """
        Resolves the root node of this plan if it is a DesignatorNode

        :return: The resolved designator
        """
        if isinstance(self.root, DesignatorNode):
            return self.root.designator_ref.resolve()

    def flattened_parameters(self):
        """
        The atomic parameter of this plan, as dict with paths as keys and the atomic type as value

        :return: A dict of the atomic types
        """
        result = {}
        for node in self.nodes:
            if isinstance(node, DesignatorNode):
                result.update(node.flattened_parameters())
        return result

    def re_perform(self):
        for child in self.root.recursive_children:
            if child.is_leaf:
                child.perform()

    @property
    def actions(self) -> List[ActionNode]:
        return list(filter(None, [node if type(node) is ActionNode else None for node in self.nodes]))

    def plot(self):
        """
        Plots the plan using matplotlib and networkx. The plan is plotted as a tree with the root node at the bottom and
        the children nodes above.
        """
        import matplotlib.pyplot as plt

        # Create a new figure
        plt.figure(figsize=(12, 8))

        # Use spring layout for node positioning
        pos = nx.drawing.bfs_layout(self, start=self.root, align='horizontal')

        # Draw nodes (bodies)
        nx.draw_networkx_nodes(self, pos,
                               node_color='lightblue',
                               node_size=2000)

        # Draw edges (connections)
        edges = self.edges(data=True)
        nx.draw_networkx_edges(self, pos,
                               edge_color='gray',
                               arrows=True,
                               arrowsize=20)

        # Add link names as labels
        labels = {node: str(node) for node in self.nodes()}
        nx.draw_networkx_labels(self, pos, labels)

        plt.title("Plan Structure")
        plt.axis('off')  # Hide axes
        plt.show()

    def add_on_start_callback(self, callback: Callable[[ResolvedActionNode], None],
                              action_type: Type[ActionDescription]):
        """
        Adds a callback to be called when an action of the given type is started.

        :param callback: The callback to be called
        :param action_type: The type of the action
        """
        if not self.on_start_callback:
            self.on_start_callback = {}
        if action_type not in self.on_start_callback:
            self.on_start_callback[action_type] = []
        self.on_start_callback[action_type].append(callback)

    def add_on_end_callback(self, callback: Callable[[ResolvedActionNode], None], action_type: Type[ActionDescription]):
        """
        Adds a callback to be called when an action of the given type is ended.

        :param callback: The callback to be called
        :param action_type: The type of the action
        """
        if not self.on_end_callback:
            self.on_end_callback = {}
        if action_type not in self.on_end_callback:
            self.on_end_callback[action_type] = []
        self.on_end_callback[action_type].append(callback)

    def remove_on_start_callback(self, callback: Callable[[ResolvedActionNode], None],
                                 action_type: Type[ActionDescription]):
        """
        Removes a callback to be called when an action of the given type is started.

        :param callback: The callback to be removed
        :param action_type: The type of the action
        """
        if self.on_start_callback and action_type in self.on_start_callback:
            self.on_start_callback[action_type].remove(callback)

    def remove_on_end_callback(self, callback: Callable[[ResolvedActionNode], None],
                               action_type: Type[ActionDescription]):
        """
        Removes a callback to be called when an action of the given type is ended.

        :param callback: The callback to be removed
        :param action_type: The type of the action
        """
        if self.on_end_callback and action_type in self.on_end_callback:
            self.on_end_callback[action_type].remove(callback)


def managed_node(func: Callable) -> Callable:
    """
    Decorator which manages the state of a node, including the start and end time, status and reason of failure as well
    as the setting of the current node in the plan.

    :param func: Reference to the perform function of the node
    :return: The wrapped perform function
    """
    def wrapper(node: DesignatorNode) -> Any:
        node.status = TaskStatus.RUNNING
        node.start_time = datetime.now()
        on_start_callbacks = (node.plan.on_start_callback.get(node.action, []) +
                              node.plan.on_start_callback.get(ActionDescription, []))
        on_end_callbacks = (node.plan.on_end_callback.get(node.action, []) +
                            node.plan.on_end_callback.get(ActionDescription, []))
        for call_back in on_start_callbacks:
            call_back(node)
        result = None
        try:
            node.plan.current_node = node
            result = func(node)
            node.status = TaskStatus.SUCCEEDED
            node.result = result
        except PlanFailure as e:
            node.status = TaskStatus.FAILED
            node.reason = e
            raise e
        finally:
            node.end_time = datetime.now()
            node.plan.current_node = node.parent
            for call_back in on_end_callbacks:
                call_back(node)
            return result

    return wrapper


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

    result: Any = None
    """
    Result from the execution of this node
    """

    @property
    def parent(self) -> PlanNode:
        """
        The parent node of this node, None if this is the root node

        :return: The parent node
        """
        return list(self.plan.predecessors(self))[0] if list(self.plan.predecessors(self)) else None

    @property
    def children(self) -> List[PlanNode]:
        """
        All children nodes of this node

        :return:  A list of child nodes
        """
        return list(self.plan.successors(self))

    @property
    def recursive_children(self) -> List[PlanNode]:
        """
        Recursively lists all children and their children.

        :return: A list of all nodes below this node
        """
        return list(nx.descendants(self.plan, self))

    @property
    def subtree(self):
        """
        Creates a new plan with this node as the new root

        :return: A new plan
        """
        return nx.subgraph(self.plan, self.recursive_children + [self])

    @property
    def all_parents(self) -> List[PlanNode]:
        """
        Returns all nodes above this node until the root node

        :return: A list of all nodes above this
        """
        return list(nx.ancestors(self.plan, self))

    @property
    def is_leaf(self) -> bool:
        """
        Returns True if this node is a leaf node

        :return: True if this node is a leaf node
        """
        return self.children == []

    def flattened_parameters(self):
        """
        The atomic types pf this node as dict

        :return: The flattened parameter
        """
        pass

    def __hash__(self):
        return id(self)

    def perform(self, *args, **kwargs):
        pass

    def interrupt(self):
        """
        Interrupts the execution of this node and all nodes below
        """
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
        return f"<{self.designator_ref.performable.__name__}>"

    def flattened_parameters(self):
        return self.designator_ref.performable.flattened_parameters()


@dataclass
class ActionNode(DesignatorNode):
    """
    A node in the plan representing an ActionDesignator description
    """
    action_iter: Iterator[ActionDescription] = None
    """
    Iterator over the current evaluation state of the ActionDesignator Description
    """

    def __hash__(self):
        return id(self)

    @managed_node
    def perform(self):
        """
        Performs this node by resolving the ActionDesignator description to the next resolution and then performing the
        result.

        :return: Return value of the resolved action node
        """
        if not self.action_iter:
            self.action_iter = iter(self.designator_ref)
        resolved_action = next(self.action_iter)
        resolved_action_node = ResolvedActionNode(designator_ref=resolved_action, action=resolved_action.__class__, )
        self.plan.add_edge(self, resolved_action_node)

        return resolved_action_node.perform()

    def __repr__(self, *args, **kwargs):
        return f"<{self.designator_ref.performable.__name__}>"


@dataclass
class ResolvedActionNode(DesignatorNode):
    """
    A node representing a resolved ActionDesignator with fully specified parameters
    """
    designator_ref: ActionDescription = None

    def __hash__(self):
        return id(self)

    @managed_node
    def perform(self):
        """
        Performs this node by performing the resolved action designator in zit

        :return: The return value of the resolved ActionDesignator
        """
        return self.designator_ref.perform()

    def __repr__(self, *args, **kwargs):
        return f"<Resolved {self.designator_ref.__class__.__name__}>"


@dataclass
class MotionNode(DesignatorNode):
    """
    A node in the plan representing a MotionDesignator
    """
    designator_ref: BaseMotion = None
    """
    Reference to the MotionDesignator
    """

    def __hash__(self):
        return id(self)

    def wait(self):
        continue_execution = False
        while not continue_execution:
            all_parents_status = [parent.status for parent in self.all_parents]
            if TaskStatus.SLEEPING not in all_parents_status:
                continue_execution = True
            time.sleep(0.1)

    @managed_node
    def perform(self):
        """
        Performs this node by performing the respective MotionDesignator. Additionally, checks if one of the parents has
        the status INTERRUPTED and aborts the perform if that is the case.

        :return: The return value of the Motion Designator
        """
        all_parents_status = [parent.status for parent in self.all_parents]
        if TaskStatus.INTERRUPTED in all_parents_status:
            return
        elif TaskStatus.SLEEPING in all_parents_status:
            self.wait()
        return self.designator_ref.perform()

    def __repr__(self, *args, **kwargs):
        return f"<{self.designator_ref.__class__.__name__}>"


def with_plan(func: Callable) -> Callable:
    """
    Decorator which wrapps the decorated designator into a node, creates a new plan with the node as root and returns
    the plan.

    :param func: The decorator which should be inserted into a plan
    :return: A plan with the designator as root node
    """

    def wrapper(*args, **kwargs) -> Plan:
        designator = func(*args, **kwargs)
        if designator.__class__.__name__ == "PartialDesignator":
            node = ActionNode(designator_ref=designator, action=designator.performable, kwargs=designator.kwargs)
        else:
            kwargs = dict(inspect.signature(func).bind(*args, **kwargs).arguments)
            node = MotionNode(designator_ref=designator, action=designator.__class__, kwargs=kwargs)
        plan = Plan(root=node)
        if Plan.current_plan:
            Plan.current_plan.mount(plan, Plan.current_plan.current_node)
        return plan

    return wrapper
