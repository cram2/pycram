"""Implementation of task.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.

TODO
"""

# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import inspect
import time
import pybullet

from graphviz import Digraph
from typing import List, Dict, Optional, Tuple, Callable, Any, Union
from enum import Enum, auto
from .taskpath import TaskPath
from .bullet_world import BulletWorld
import anytree
import datetime


TASK_TREE = None
CURRENT_TASK_TREE_NODE = None
task_tree = None
current_task_tree_node = None


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

    # maybe pp stands for pretty print ???
    def pp(self) -> str:
        return str(self)

    def __str__(self) -> str:
        return "%s(%s)" % (self.function.__name__, ", ".join(["%s = %s" % (key, str(value))
                                                              for key, value in self.kwargs.items()]))

    def important_information(self):
        return {"function": self.function.__name__, "kwargs": {**self.kwargs}}


class NoOperation(Code):
    """
    Convenience class that represents no operation as code.
    """
    def __init__(self):

        # default no operation
        def no_operation(): return None

        # initialize a code block that does nothing
        super().__init__(no_operation)


class TaskTreeNode2(anytree.NodeMixin):
    """Refactoring of TaskTreeNode"""

    def __init__(self, code: Code = NoOperation(), parent=None, children=None):
        super().__init__()
        self.code: Code = code
        self.exec_step: int = 0
        self.status: TaskStatus = TaskStatus.CREATED
        self.start_time: Optional[datetime.datetime] = None
        self.end_time: Optional[datetime.datetime] = None
        self.parent = parent

        if children:
            self.children = children

    def execute(self):
        """Try executing the task of this node."""
        result = None
        try:
            result = self.code.execute()
        finally:
            self.exec_step += 1
        return result

    def __str__(self):
        return "Code: %s " \
               "start_time: %s"\
               "" % (str(self.code), self.start_time)

    def __repr__(self):
        return str(self.code)


class TaskTreeNode:
    def __init__(self, code=None, parent: Optional[TaskTreeNode] = None, path: Optional[str] = ""):
        self.code = code
        self.parent: Optional[TaskTreeNode] = parent
        self.children: List[TaskTreeNode] = []
        self.path: str = path
        self.exec_parent_prev: Optional[TaskTreeNode] = None
        self.exec_parent_next: Optional[TaskTreeNode] = None
        self.exec_child_prev: Optional[TaskTreeNode] = None
        self.exec_child_next: Optional[TaskTreeNode] = None
        self.exec_step = 0
        self.status: TaskStatus = TaskStatus.CREATED
        self.failure = None
        self.start_time = None
        self.end_time = None

    def important_information(self):
        return {
            "status": self.status,
            "start_time": self.start_time,
            "end_time": self.end_time,
            "failure": self.failure,
            "path": self.path,
            "code": self.code.important_information()
        }

    def pp(self) -> str:
        # return nothing, when whole exec_child_tree is deleted
        if type(self.code) is NoOperation:
            return ""

        pretty = ""

        # Add previous siblings
        if self.exec_child_prev:
            pretty += self.exec_child_prev.pp()

        # If not root, add line break
        if self.parent:
            pretty += "\n"

        # Indent 2 spaces for each level of depth
        parent = self.parent
        while parent:
            pretty += "  "
            parent = parent.parent

        # if not root, add branch symbol thingy
        if self.parent:
            pretty += "|- "

        # finally add function name
        pretty += self.code.function.__name__ + " (" + self.path + ")"

        # add pp for all children
        for c in self.children:
            pretty += c.pp()

        # Add next siblings
        if self.exec_child_next:
            pretty += self.exec_child_next.pp()

        return pretty

    def generate_dot(self, dot: Optional[Digraph] = None, verbose: Optional[bool] = True) -> Digraph:
        if not dot:
            dot = Digraph()
        if self.exec_child_prev:
            dot = self.exec_child_prev.generate_dot(dot, verbose)

        if type(self.code) is not NoOperation:
            if verbose:
                label = "\n".join([self.path.split("/")[-1], self.code.pp(), str(self.status), str(self.start_time), str(self.end_time)])
            else:
                label = "\n".join([self.path.split("/")[-1], str(self.status)])
            if self.failure:
                label = "\n".join([label, str(self.failure)])
            dot.node(self.path, label, shape="box", style="filled")
            if self.parent:
                dot.edge(self.parent.path, self.path)
            for c in self.children:
                dot = c.generate_dot(dot, verbose)

        if self.exec_child_next:
            dot = self.exec_child_next.generate_dot(dot, verbose)
        return dot

    def execute(self) -> Any:
        self.execute_prev()
        self.exec_step = 0
        self.status = TaskStatus.RUNNING
        self.start_time = time.time()
        global CURRENT_TASK_TREE_NODE
        CURRENT_TASK_TREE_NODE = self
        try:
            result = self.code.execute()
            self.status = TaskStatus.SUCCEEDED
        except Exception as e:
            self.status = TaskStatus.FAILED
            self.failure = e
            raise e
        finally:
            CURRENT_TASK_TREE_NODE = CURRENT_TASK_TREE_NODE.parent
            self.end_time = time.time()
        self.execute_next()
        return result

    def execute_prev(self) -> Any:
        if self.exec_child_prev:
            return self.exec_child_prev.execute()

    def execute_next(self) -> Any:
        if self.exec_child_next:
            return self.exec_child_next.execute()

    def execute_child(self) -> Any:
        try:
            result = self.children[self.exec_step].execute()
        finally:
            self.exec_step += 1
        return result

    def delete(self) -> None:
        # Not an ExecNode
        if not (self.exec_parent_prev or self.exec_parent_next):
            self.code = NoOperation()
        # ExecNode and has one or two children
        elif self.exec_child_prev or self.exec_child_next:
            # Relink exec_children if necessary
            if self.exec_child_prev and self.exec_child_next:
                self.exec_child_prev.insert_after(self.exec_child_next)
                link_child = self.exec_child_prev
                link_child.exec_parent_next = None
            elif self.exec_child_prev:
                link_child = self.exec_child_prev
                link_child.exec_parent_next = None
            else:
                link_child = self.exec_child_next
                link_child.exec_parent_prev = None
            # Link parent to new child
            if self.exec_parent_prev:
                self.exec_parent_prev.set_exec_child_next(link_child)
            elif self.exec_parent_next:
                self.exec_parent_next.set_exec_child_prev(link_child)
        # ExecNode but no exec_children
        else:
            if self.exec_parent_prev:
                self.exec_parent_prev.exec_child_next = None
            elif self.exec_parent_next:
                self.exec_parent_next.exec_child_prev = None

    def copy(self) -> TaskTreeNode:
        copy_node = TaskTreeNode(self.code)
        copy_node.children = self.children
        return copy_node

    def add_child(self, child: TaskTreeNode) -> None:
        self.children.append(child)
        child.fix_path()

    def set_exec_child_prev(self, child: TaskTreeNode) -> None:
        self.exec_child_prev = child
        child.exec_parent_next = self
        child.exec_child_prev = None
        child.parent = self.parent
        child.fix_path()

    def set_exec_child_next(self, child: TaskTreeNode) -> None:
        self.exec_child_next = child
        child.exec_parent_prev = self
        child.exec_parent_next = None
        child.parent = self.parent
        child.fix_path()

    # TODO(?): Remove noop nodes from list?
    def gen_children_list(self) -> List[TaskTreeNode]:
        result = []
        for c in self.children:
            result += c.gen_sibling_list()
        return result

    def gen_sibling_list(self) -> List[TaskTreeNode]:
        """
        Generate a list of exec_tree children/siblings
        :return:
        """
        if self.exec_child_prev and self.exec_child_next:
            return self.exec_child_prev.gen_sibling_list() + [self] + self.exec_child_next.gen_sibling_list()
        elif self.exec_child_prev:
            return self.exec_child_prev.gen_sibling_list() + [self]
        elif self.exec_child_next:
            return [self] + self.exec_child_next.gen_sibling_list()
        else:
            return [self]

    def fix_path(self) -> None:
        if self.parent and self.parent.path:
            # fix path
            max_i = -1
            for c in self.parent.gen_children_list():
                if c is self:
                    continue
                if c.code.function == self.code.function:
                    max_i = max(max_i, int(c.path[-1]))  # TODO: This stops working for indices over 9
            self.path = '/'.join([self.parent.path, self.code.function.__name__]) + str(max_i+1)

            # fix children
            for c in self.gen_children_list():
                c.fix_path()
        else:
            self.parent.fix_path()

    def get_child_by_path(self, path: Union[str, TaskPath]) -> Union[None, TaskTreeNode]:
        if type(path) is str:
            path = TaskPath(path)
        children = self.gen_children_list()
        children_with_name = list(filter(lambda x: x.code.function.__name__ == path.path[0].name, children))
        try:
            result = children_with_name[path.path[0].n]
            if len(path) > 1:
                return result.get_child_by_path(path.cut_off_head())
            else:
                return result
        except IndexError:
            raise IndexError("Node at path {} does not have child with name {} at number {}.".format(self.path, path.path[0].name, path.path[0].n))

    def insert_before(self, node: TaskTreeNode) -> None:
        if self.exec_child_prev:
            n = self.exec_child_prev
            while n.exec_child_next:
                n = n.exec_child_next
            n.set_exec_child_next(node)
        else:
            self.set_exec_child_prev(node)
        node.fix_path()

    def insert_after(self, node: TaskTreeNode) -> None:
        if self.exec_child_next:
            n = self.exec_child_next
            while n.exec_child_prev:
                n = n.exec_child_prev
            n.set_exec_child_prev(node)
        else:
            self.set_exec_child_next(node)
        node.fix_path()

    def replace_child(self, node: TaskTreeNode, new_node: TaskTreeNode) -> bool:
        for i in range(len(self.children)):
            child = self.children[i]
            if child is node:
                self.children[i] = new_node
                new_node.parent = self
                new_node.exec_child_prev = child.exec_child_prev
                new_node.exec_child_next = child.exec_child_next
                new_node.fix_path()
                return True
            elif self.replace_exec_child(node, new_node):
                return True
        return False

    def replace_exec_child(self, node: TaskTreeNode, new_node: TaskTreeNode) -> bool:
        if self.exec_child_prev is node:
            new_node.exec_child_prev = self.exec_child_prev.exec_child_prev
            new_node.exec_child_next = self.exec_child_prev.exec_child_next
            self.exec_child_prev = new_node
            return True
        if self.exec_child_next is node:
            new_node.exec_child_prev = self.exec_child_next.exec_child_prev
            new_node.exec_child_next = self.exec_child_next.exec_child_next
            self.exec_child_next = new_node
            return True
        if self.exec_child_prev and self.exec_child_prev.replace_exec_child(node, new_node):
            return True
        if self.exec_child_next and self.exec_child_next.replace_exec_child(node, new_node):
            return True
        return False

    def params(self):
        return self.code.args, self.code.kwargs

    def delete_following(self, inclusive: Optional[bool] = False) -> None:
        kill = False
        for c in self.parent.gen_children_list():
            if kill:
                c.delete()
            if c is self:
                kill = True
        if inclusive:
            self.delete()

    def delete_previous(self, inclusive: Optional[bool] = False) -> None:
        kill = False
        children = self.parent.gen_children_list()
        children.reverse()
        for c in children:
            if kill:
                c.delete()
            if c is self:
                kill = True
        if inclusive:
            self.delete()

    def get_successful_params_ctx_after(self, name: str, ctx: str) -> List:
        params = []
        noi = None  # Node of Interest
        for c in self.gen_children_list():
            if c.code.function.__name__ == name and c.status == TaskStatus.SUCCEEDED:
                noi = c
            if noi and c.code.function.__name__ == ctx and c.status == TaskStatus.SUCCEEDED:
                params.append(noi.params())
                noi = None
        return params


class SimulatedTaskTree:
    def __enter__(self):
        global TASK_TREE
        global CURRENT_TASK_TREE_NODE
        self.suspended_tree = TASK_TREE
        self.suspended_current_node = CURRENT_TASK_TREE_NODE
        self.world_state, self.objects2attached = BulletWorld.current_bullet_world.save_state()
        self.simulated_root = TaskTreeNode(code=Code("Simulation"), path="dream")
        TASK_TREE = self.simulated_root
        CURRENT_TASK_TREE_NODE = self.simulated_root
        pybullet.addUserDebugText("Simulating...", [0, 0, 1.75], textColorRGB=[0, 0, 0],
                                  parentObjectUniqueId=1, lifeTime=0)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        global TASK_TREE
        global CURRENT_TASK_TREE_NODE
        TASK_TREE = self.suspended_tree
        CURRENT_TASK_TREE_NODE = self.suspended_current_node
        BulletWorld.current_bullet_world.restore_state(self.world_state, self.objects2attached)
        pybullet.removeAllUserDebugItems()

    def get_successful_params_ctx_after(self, name: str, ctx: str) -> List:
        # This only works for very specific contexts
        # And only if there are no other actions with NAME before the context of interest
        params = []
        noi = None  # Node of Interest
        for c in self.simulated_root.gen_children_list():
            if c.code.function.__name__ == name and c.status == TaskStatus.SUCCEEDED:
                noi = c
            if noi and c.code.function.__name__ == ctx and c.status == TaskStatus.SUCCEEDED:
                params.append(noi.params())
                noi = None
        return params


def move_node_to_path(node: TaskTreeNode, path: TaskPath, where: Optional[int] = 1) -> Union[None, TaskTreeNode]:
    """
    Move a node to a path.
    :param node: TaskTreeNode
    :param path: TaskTreePath
    :param where: An integer indicating where to put the node: -1=before the node, 0=instead of the node, 1=after the node
    """
    global TASK_TREE
    node_at_path = TASK_TREE.get_child_by_path(path)
    node_copy = node.copy()
    for c in node_copy.gen_children_list():
        c.parent = None
    node_copy.children = []
    if where == -1:
        node.delete()
        node_at_path.insert_before(node_copy)
    elif where == 1:
        node.delete()
        node_at_path.insert_after(node_copy)
    elif where == 0:
        node.delete()
        node_at_path.parent.replace_child(node_at_path, node_copy)
    else:
        print("WARN - Moving node: where parameter is not properly set.")
        return
    return node_copy


def get_successful_params(nodes: List[TaskTreeNode]) -> List:
    p = []
    for n in nodes:
        if n.status == TaskStatus.SUCCEEDED:
            p.append(n.params())
    return p


def with_tree(fun: Callable) -> Callable:
    """Decorator that records the function name, arguments and execution metadata in the task tree.

    :param fun: The function to record the data from.
    """
    def handle_tree(*args, **kwargs):

        # get the global task tree
        global TASK_TREE
        global task_tree

        # get the current node in the global task tree
        global CURRENT_TASK_TREE_NODE
        global current_task_tree_node

        # create the code object that gets executed
        code = Code(fun, inspect.getcallargs(fun, *args, **kwargs))

        # if there is no global task tree yet
        if CURRENT_TASK_TREE_NODE is None:

            # create the root node
            TASK_TREE = TaskTreeNode(code, None, fun.__name__)
            task_tree = TaskTreeNode2(code)

            # update pointer to current node
            CURRENT_TASK_TREE_NODE = TASK_TREE
            current_task_tree_node = task_tree

            # execute the function
            result = CURRENT_TASK_TREE_NODE.execute()

        # if a global task tree already exists
        else:

            # if the number of children is not bigger than the number of tasks that should be executed
            if len(CURRENT_TASK_TREE_NODE.children) <= CURRENT_TASK_TREE_NODE.exec_step:

                # create a new node
                new_node = TaskTreeNode(code, CURRENT_TASK_TREE_NODE,
                                        '/'.join([CURRENT_TASK_TREE_NODE.path, fun.__name__]))
                # create a new node and automatically add it to the tree
                new_node2 = TaskTreeNode2(code, parent=current_task_tree_node)

                # add it to the tree
                CURRENT_TASK_TREE_NODE.add_child(new_node)

            # execute the function
            result = CURRENT_TASK_TREE_NODE.execute_child()
        return result

    # return function to execute for decorator wrapping
    return handle_tree
