"""Implementation of TaskTrees using anytree."""

# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations
import datetime
import inspect
import logging

from anytree import RenderTree
from anytree.exporter import DotExporter
from typing_extensions import List, Optional, Callable, Dict, Type, TYPE_CHECKING
import anytree
from .datastructures.world import World
from .helper import Singleton
from .failures import PlanFailure
from .datastructures.enums import TaskStatus
from .datastructures.dataclasses import Color

if TYPE_CHECKING:
    from .designators.action_designator import ActionDescription as Action


class NoOperation:

    def perform(self):
        ...

    def __repr__(self):
        return "NoOperation"


class TaskTreeNode(anytree.NodeMixin):
    """
    TaskTreeNode represents one function that was called during a pycram plan.
    Additionally, meta information is stored.
    """

    action: Optional[Action]
    """
    The action and that is performed or None if nothing was performed
    """

    status: TaskStatus
    """
    The status of the node from the TaskStatus enum.
    """

    start_time: Optional[datetime.datetime]
    """
    The starting time of the function, optional
    """

    end_time: Optional[datetime.datetime]
    """
    The ending time of the function, optional
    """

    reason: Optional[Exception] = None
    """
    The reason of failure if the action failed.
    """

    def __init__(self, action: Optional[Action] = None, parent: Optional[TaskTreeNode] = None,
                 children: Optional[List[TaskTreeNode]] = None, reason: Optional[Exception] = None):
        """
        Create a TaskTreeNode

        :param action: The action and that is performed, defaults to None
        :param parent: The parent function of this function. None if this the parent, optional
        :param children: An iterable of TaskTreeNode with the ordered children, optional
        """
        super().__init__()

        self.action = action
        self.status = TaskStatus.CREATED
        self.start_time = datetime.datetime.now()
        self.end_time = None
        self.parent = parent
        self.reason: Optional[Exception] = reason

        if children:
            self.children = children

    @property
    def name(self):
        return str(self)

    def __str__(self):
        return "Code: %s \n " \
               "Status: %s \n " \
               "start_time: %s \n " \
               "end_time: %s \n " \
               "" % (str(self.action), self.status, self.start_time, self.end_time)

    def __repr__(self):
        return str(self.action.__class__.__name__)

    def __len__(self):
        """Get the number of nodes that are in this subtree."""
        return 1 + sum([len(child) for child in self.children])


class SimulatedTaskTree:
    """TaskTree for execution in a 'new' simulation."""

    def __enter__(self):
        """At the beginning of a with statement the current task tree and world will be suspended and remembered.
        Fresh structures are then available inside the with statement."""
        global task_tree

        self.suspended_tree = task_tree
        self.world_state = World.current_world.save_state()
        self.simulated_root = TaskTree()
        task_tree = self.simulated_root
        World.current_world.add_text("Simulating...", [0, 0, 1.75], color=Color.from_rgb([0, 0, 0]),
                                     parent_object_id=1)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Restore the old state at the end of a with block.
        """
        global task_tree
        task_tree = self.suspended_tree
        World.current_world.restore_state(self.world_state)
        World.current_world.remove_text()


class TaskTree(metaclass=Singleton):
    """
    TaskTree represents the tree of functions that were called during a pycram plan. Consists of TaskTreeNodes.
    Must be a singleton.
    """

    on_start_callbacks: Optional[Dict[Type[Action], List[Callable[[TaskTreeNode], None]]]] = None
    """
    Callbacks that are called when a node with a specific action is inserted.
    """
    on_end_callbacks: Optional[Dict[Type[Action], List[Callable[[TaskTreeNode], None]]]] = None
    """
    Callbacks that are called when an action related to specific node has ended.
    """

    def __init__(self):
        """
        Create a new TaskTree with a root node.
        """
        self.root = TaskTreeNode()
        self.current_node = self.root
        self.name = "TaskTree"
        self.on_start_callbacks = {}
        self.on_end_callbacks = {}

    def add_callback(self, callback: Callable[[TaskTreeNode], None], action_type: Optional[Type[Action]] = None,
                     on_start: bool = True):
        """
        Add a callback that is called when a node with a specific action is inserted.

        :param callback: The callback to be called.
        :param action_type: The action type that triggers the callback, if None, will be called for all actions.
        :param on_start: Rather to call the callback on the start or the end of the action.
        """
        callbacks = self.on_start_callbacks if on_start else self.on_end_callbacks
        if action_type in callbacks:
            callbacks[action_type].append(callback)
        else:
            callbacks[action_type] = [callback]

    @property
    def children(self):
        return self.root.children

    def __len__(self):
        """
        Get the number of nodes that are in this TaskTree.

        :return: The number of nodes.
        """
        return len(self.root.children)

    def reset_tree(self):
        """
        Reset the current task tree to an empty root (NoOperation) node.
        """
        self.root = TaskTreeNode()
        self.root.start_time = datetime.datetime.now()
        self.root.status = TaskStatus.RUNNING
        self.current_node = self.root

    def add_node(self, action: Optional[Action] = None) -> TaskTreeNode:
        """
        Add a new node to the task tree and make it the current node.

        :param action: The action that is performed in this node.
        :return: The new node.
        """
        new_node = TaskTreeNode(action=action, parent=self.current_node)
        self.current_node = new_node
        self.call_callbacks(new_node)
        return new_node

    def call_callbacks(self, new_node: TaskTreeNode, on_start: bool = True):
        """
        Call all callbacks for the given action.

        :param new_node: The new node that was added.
        :param on_start: Rather to call the on_start_callbacks or the on_end_callbacks.
        """
        action: Action = new_node.action
        callbacks = self.on_start_callbacks if on_start else self.on_end_callbacks
        if callbacks:
            callback_actions = list(callbacks.keys())
            if action.__class__ in callback_actions:
                for callback in callbacks[action.__class__]:
                    callback(new_node)
            if None in callback_actions:
                for callback in callbacks[None]:
                    callback(new_node)

    @staticmethod
    def render(file_name: str):
        """
        Render the task tree to a dot file and a png file.

        :param file_name: The name of the file without extension.
        """
        def task_node_name(node):
            start_time = node.start_time.time() if node.start_time else node.start_time
            end_time = node.end_time.time() if node.end_time else node.end_time
            return f"Code: {node.action} \n" \
                   f"Status: {node.status} \n" \
                   f"start_time: {datetime_to_str(start_time)}\n" \
                   f"end_time: {datetime_to_str(end_time)}\n"

        def datetime_to_str(time_):
            if not time_:
                return None
            return f"{time_.minute}:{time_.second}:{int(time_.microsecond * 0.001)}"

        for pre, _, node in RenderTree(task_tree.root):
            print(f"{pre}{node.weight if hasattr(node, 'weight') and node.weight else ''} {node.__str__()}")

        de = DotExporter(task_tree.root,
                         nodenamefunc=task_node_name,
                         )
        de.to_dotfile(f"{file_name}.dot")
        de.to_picture(f"{file_name}.png")


task_tree = TaskTree()
"""Current TaskTreeNode"""


def with_tree(fun: Callable) -> Callable:
    """
    Decorator that records the function name, arguments and execution metadata in the task tree.

    :param fun: The function to record the data from.
    """

    def handle_tree(*args, **kwargs):
        # get the task tree
        global task_tree
        x = task_tree

        # parse keyword arguments
        keyword_arguments = inspect.getcallargs(fun, *args, **kwargs)

        # try to get self object since this represents the action object
        action = keyword_arguments.get("self", None)

        # create the task tree node
        task_tree.add_node(action)

        # Try to execute the task
        try:
            task_tree.current_node.status = TaskStatus.CREATED
            task_tree.current_node.start_time = datetime.datetime.now()
            result = fun(*args, **kwargs)

            # if it succeeded set the flag
            task_tree.current_node.status = TaskStatus.SUCCEEDED

        # iff a PlanFailure occurs
        except PlanFailure as e:

            # log the error and set the flag
            logging.exception("Task execution failed at %s. Reason %s" % (repr(task_tree.current_node), e))
            task_tree.current_node.reason = e
            task_tree.current_node.status = TaskStatus.FAILED
            raise e

        finally:
            if task_tree.current_node.parent is not None:
                task_tree.current_node.end_time = datetime.datetime.now()
                task_tree.call_callbacks(task_tree.current_node, on_start=False)
                task_tree.current_node = task_tree.current_node.parent

        return result

    return handle_tree
