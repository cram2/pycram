from __future__ import annotations

import inspect
import time
from dataclasses import field, dataclass
from datetime import datetime

import networkx as nx
from bokeh.models.annotations.labels import LabelSet
from bokeh.models.sources import ColumnDataSource
from typing_extensions import Optional, Callable, Any, Dict, List, Iterable, TYPE_CHECKING, Type, Tuple, Iterator

from .datastructures.enums import TaskStatus
from .external_interfaces import giskard
from .failures import PlanFailure
from .has_parameters import leaf_types
from .ros import loginfo

if TYPE_CHECKING:
    from .designator import BaseMotion, ActionDescription


class Plan(nx.DiGraph):
    """
    Represents a plan structure, typically a tree, which can be changed at any point in time. Performing the plan will
    traverse the plan structure in depth first order and perform each PlanNode
    """
    current_plan: Plan = None

    on_start_callback: Dict[Optional[Type[ActionDescription]], List[Callable]] = {}
    on_end_callback: Dict[Optional[Type[ActionDescription]], List[Callable]] = {}

    def __init__(self, root: PlanNode):
        super().__init__()
        self.root: PlanNode = root
        self.add_node(self.root)
        self.current_node: PlanNode = self.root
        self.on_start_callback = {}
        self.on_end_callback = {}

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

    @classmethod
    def add_on_start_callback(cls, callback: Callable[[ResolvedActionNode], None],
                              action_type: Optional[Type[ActionDescription]] = None):
        """
        Adds a callback to be called when an action of the given type is started.

        :param callback: The callback to be called
        :param action_type: The type of the action, if None, the callback will be called for all actions
        """
        if not cls.on_start_callback:
            cls.on_start_callback = {}
        if action_type not in cls.on_start_callback:
            cls.on_start_callback[action_type] = []
        cls.on_start_callback[action_type].append(callback)

    @classmethod
    def add_on_end_callback(cls, callback: Callable[[ResolvedActionNode], None],
                            action_type: Optional[Type[ActionDescription]] = None):
        """
        Adds a callback to be called when an action of the given type is ended.

        :param callback: The callback to be called
        :param action_type: The type of the action
        """
        if not cls.on_end_callback:
            cls.on_end_callback = {}
        if action_type not in cls.on_end_callback:
            cls.on_end_callback[action_type] = []
        cls.on_end_callback[action_type].append(callback)

    @classmethod
    def remove_on_start_callback(cls, callback: Callable[[ResolvedActionNode], None],
                                 action_type: Optional[Type[ActionDescription]] = None):
        """
        Removes a callback to be called when an action of the given type is started.

        :param callback: The callback to be removed
        :param action_type: The type of the action
        """
        if cls.on_start_callback and action_type in cls.on_start_callback:
            cls.on_start_callback[action_type].remove(callback)

    @classmethod
    def remove_on_end_callback(cls, callback: Callable[[ResolvedActionNode], None],
                               action_type: Optional[Type[ActionDescription]] = None):
        """
        Removes a callback to be called when an action of the given type is ended.

        :param callback: The callback to be removed
        :param action_type: The type of the action
        """
        if cls.on_end_callback and action_type in cls.on_end_callback:
            cls.on_end_callback[action_type].remove(callback)

    def _create_pure_networkx_graph(self, attributes: List[str]) -> nx.DiGraph[int]:
        """
        Creates a pure networkx graph of this plan and adds the given attributes of nodes as networkx Node attributes.

        :param attributes: A list of attributes from the nodes which should be contained in the returned graph
        :return: A NetworkX graph of the hash values of the PlanNodes
        """
        hash_nodes = {hash(node): node for node in self.nodes}
        edges = [(hash(source), hash(target)) for source, target in self.edges]
        graph = nx.DiGraph()
        graph.add_nodes_from(hash_nodes.keys())
        graph.add_edges_from(edges)

        for v in graph:
            for attr in attributes:
                graph.nodes[v][attr] = str(getattr(hash_nodes[v], attr))
        return graph

    def plot_bokeh(self, attributes: List[str] = None):
        attributes = attributes or ["status", "start_time"]
        from bokeh.plotting import figure, from_networkx, show
        from bokeh.models import (HoverTool, NodesAndLinkedEdges)

        p = figure(x_range=(-2, 2), y_range=(-2, 2),
                   width=1700, height=950,
                   x_axis_location=None, y_axis_location=None, toolbar_location=None,
                   title="Plan Visualization", background_fill_color="#efefef", )
        node_hover_tool = HoverTool(tooltips=[("status", "@status"), ("start", "@start_time")])
        p.add_tools(node_hover_tool)

        p.grid.grid_line_color = None
        p.add_layout(self._create_labels())

        graph = from_networkx(self._create_pure_networkx_graph(attributes), nx.drawing.bfs_layout,
                              start=hash(self.root), align='horizontal')
        graph.selection_policy = NodesAndLinkedEdges()
        graph.inspection_policy = NodesAndLinkedEdges()

        graph.node_renderer.glyph.update(size=20, fill_color="lightblue")

        p.renderers.append(graph)

        show(p, new="same")

    def _create_labels(self) -> LabelSet:
        """
        Creates a label set for the plan visualization. Labels are the PlanNode.action attribute.

        :return: A LabelSet object which can be added to a bokeh plot.
        """
        from bokeh.models import ColumnDataSource, LabelSet
        hash_nodes = {hash(node): node for node in self.nodes}
        layout=nx.drawing.bfs_layout(self._create_pure_networkx_graph([]), start=hash(self.root), align='horizontal')
        x = [pose[0] for pose in layout.values()]
        y = [pose[1] for pose in layout.values()]
        name = [str(hash_nodes[node].action.__name__) for node in layout.keys()]
        label_dict = {'x': x, 'y': y, 'names': name}

        data_source = ColumnDataSource(data=label_dict)
        labels= LabelSet(x='x', y='y', text='names',
                          x_offset=-55, y_offset=10, source=data_source)
        return labels


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
        on_start_callbacks = (Plan.on_start_callback.get(node.action, []) +
                              Plan.on_start_callback.get(None, []))
        on_end_callbacks = (Plan.on_end_callback.get(node.action, []) +
                            Plan.on_end_callback.get(None, []))
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

    def resume(self):
        """
        Resumes the execution of this node and all nodes below
        """
        self.status = TaskStatus.RUNNING

    def pause(self):
        """
        Suspends the execution of this node and all nodes below.
        """
        self.status = TaskStatus.SLEEPING


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

    def flattened_parameters(self) -> Dict[str, leaf_types]:
        """
        The atomic types of the parameters of this node as dict with paths as keys and the atomic type as value.
        This resolves the parameters to its type not the actual value.

        :return: The atomic types of this action
        """
        return self.designator_ref.performable.flattened_parameters()

    def flatten(self) -> Dict[str, leaf_types]:
        """
        Flattens the parameters of this node to a dict with the parameter as  key and the value as value.

        :return: A dict of the flattened parameters
        """
        params = self.designator_ref.performable.flattened_parameters()
        for key, value in self.designator_ref.kwargs.items():
            if key in params:
                params[key] = value
        return params


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
        kwargs = {key: resolved_action.__getattribute__(key) for key in self.designator_ref.kwargs.keys()}
        resolved_action_node = ResolvedActionNode(designator_ref=resolved_action, action=resolved_action.__class__,
                                                  kwargs=kwargs)
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

    def flatten(self):
        return self.designator_ref.flatten()

    def flattened_parameters(self):
        return self.designator_ref.flattened_parameters()


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

    def flatten(self):
        return {}

    def flattened_parameters(self):
        return {}


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
