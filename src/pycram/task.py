"""Implementation of TaskTrees."""

# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import inspect
import pybullet

from graphviz import Digraph
from typing import List, Dict, Optional, Tuple, Callable, Any, Union
from enum import Enum, auto

import task
from .bullet_world import BulletWorld
import anytree
import datetime
import logging


class TaskStatus(Enum):
    """
    Enum for readable descriptions of a tasks' status.
    """
    CREATED = 0
    RUNNING = 1
    SUCCEEDED = 2
    FAILED = 3


class Code:
    """
    Represent an executed code block in a plan.
    """
    def __init__(self, function: Optional[Callable] = None,
                 kwargs: Optional[Dict] = None):
        """
        Initialize a code call
        :param function: The function that was called
        :param kwargs: The keyword arguments of the function as dict
        """
        self.function: Callable = function

        if kwargs is None:
            kwargs = dict()
        self.kwargs: Dict = kwargs

    def execute(self) -> Any:
        return self.function(**self.kwargs)

    def __str__(self) -> str:
        return "%s(%s)" % (self.function.__name__, ", ".join(["%s = %s" % (key, str(value))
                                                              for key, value in self.kwargs.items()]))

    def __eq__(self, other):
        return isinstance(other, Code) and other.function.__name__ == self.function.__name__ \
               and other.kwargs == self.kwargs

    def to_json(self) -> Dict:
        """Create a dictionary that can be json serialized."""
        return {"function": self.function.__name__, "kwargs": self.kwargs}


class NoOperation(Code):
    """
    Convenience class that represents no operation as code.
    """
    def __init__(self):

        # default no operation
        def no_operation(): return None

        # initialize a code block that does nothing
        super().__init__(no_operation)


class TaskTreeNode(anytree.NodeMixin):
    """TaskTreeNode represents one function that was called during a pycram plan.
    Additionally, meta information is stored.

    :ivar code: The function that was executed as Code object.
    :ivar status: The status of the node from the TaskStatus enum.
    :ivar start_time: The starting time of the function, optional
    :ivar end_time: The ending time of the function, optional
    """

    def __init__(self, code: Code = NoOperation(), parent: Optional[TaskTreeNode] = None,
                 children: Optional[List[TaskTreeNode]] = None):
        """
        Create a TaskTreeNode
        :param code: The function and its arguments that got called as Code object, defaults to NoOperation()
        :param parent: The parent function of this function. None if this the parent, optional
        :param children: An iterable of TaskTreeNode with the ordered children, optional
        """
        super().__init__()
        self.code: Code = code
        self.status: TaskStatus = TaskStatus.CREATED
        self.start_time: Optional[datetime.datetime] = None
        self.end_time: Optional[datetime.datetime] = None
        self.parent = parent

        if children:
            self.children = children

    def __str__(self):
        return "Code: %s " \
               "start_time: %s" \
               "Status: %s" \
               "end_time: %s" \
               "" % (str(self.code), self.start_time, self.status, self.end_time)

    def __repr__(self):
        return str(self.code)

    def __len__(self):
        """Get the number of nodes that are in this subtree."""
        return 1 + sum([len(child) for child in self.children])


class SimulatedTaskTree:
    """TaskTree for execution in a 'new' simulation."""

    def __enter__(self):
        """ At the beginning"""
        global task_tree

        def simulation(): return None

        self.suspended_tree = task_tree
        self.world_state, self.objects2attached = BulletWorld.current_bullet_world.save_state()
        self.simulated_root = TaskTreeNode(code=Code(simulation))
        task_tree = self.simulated_root
        pybullet.addUserDebugText("Simulating...", [0, 0, 1.75], textColorRGB=[0, 0, 0],
                                  parentObjectUniqueId=1, lifeTime=0)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        global task_tree
        task_tree = self.suspended_tree
        BulletWorld.current_bullet_world.restore_state(self.world_state, self.objects2attached)
        pybullet.removeAllUserDebugItems()


# initialize task tree
task_tree = None


def reset_tree():
    # Reset task tree root to no operation
    global task_tree
    task_tree = TaskTreeNode(NoOperation())
    task_tree.start_time = datetime.datetime.now()
    task_tree.status = TaskStatus.RUNNING


reset_tree()


def with_tree(fun: Callable) -> Callable:
    """Decorator that records the function name, arguments and execution metadata in the task tree.

    :param fun: The function to record the data from.
    """
    def handle_tree(*args, **kwargs):

        # get the task tree
        global task_tree

        # create the code object that gets executed
        code = Code(fun, inspect.getcallargs(fun, *args, **kwargs))

        task_tree = TaskTreeNode(code, parent=task_tree)

        try:
            task_tree.status = TaskStatus.CREATED
            task_tree.start_time = datetime.datetime.now()
            task_tree.code.execute()
            task_tree.end_time = datetime.datetime.now()
            task_tree.status = TaskStatus.SUCCEEDED

        except Exception as e:
            logging.exception("Task execution failed at %s. Reason %s" % (str(task_tree.code), e))
            task_tree.status = TaskStatus.FAILED

        task_tree = task_tree.parent

    return handle_tree
