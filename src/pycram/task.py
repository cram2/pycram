"""Implementation of TaskTrees using anytree."""

# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import datetime
import inspect
import json
import logging
from enum import Enum
from typing import List, Dict, Optional, Callable, Any

import anytree
import pybullet
import sqlalchemy.orm.session

# import pycram.designators.action_designator
from .bullet_world import BulletWorld
from .orm.task import (Code as ORMCode, TaskTreeNode as ORMTaskTreeNode)
from .plan_failures import PlanFailure


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
    Executable code block in a plan.

    :ivar function: The function (plan) that was called
    :ivar kwargs: Dictionary holding the keyword arguments of the function
    :ivar designator: The designator description if it exists, defaults to None
    """

    def __init__(self, function: Optional[Callable] = None,
                 kwargs: Optional[Dict] = None,
                 designator: Optional[pycram.designators.action_designator.ActionDesignatorDescription] = None):
        """
        Initialize a code call
        :param function: The function that was called
        :param kwargs: The keyword arguments of the function as dict
        """
        self.function: Callable = function

        if kwargs is None:
            kwargs = dict()
        self.kwargs: Dict[str, Any] = kwargs
        self.designator = designator

    def execute(self) -> Any:
        """
        Execute the code with its arguments

        :returns: Anything that the function associated with this object will return.
        """
        return self.function(**self.kwargs)

    def __str__(self) -> str:
        return "%s(%s)" % (self.function.__name__, ", ".join(["%s = %s" % (key, str(value)) for key, value in
                                                              self.kwargs.items()]))

    def __eq__(self, other):
        return isinstance(other, Code) and other.function.__name__ == self.function.__name__ \
               and other.kwargs == self.kwargs and self.designator == other.designator

    def to_json(self) -> Dict:
        """Create a dictionary that can be json serialized."""
        return {"function": self.function.__name__, "kwargs": self.kwargs_to_json()}

    def kwargs_to_json(self):
        """Try to parse the keyword arguments to json. Checks if the objects given as arguments can be serialized
        as standard object or have a to_json method. If not they are skipped. """
        result = dict()
        for keyword, argument in self.kwargs.items():
            to_json_method = getattr(argument, 'to_json', None)

            if to_json_method:
                result[keyword] = argument.to_json()

            else:
                try:
                    argument_ = json.loads(json.dumps(argument))
                    result[keyword] = argument_
                except (TypeError, OverflowError):
                    logging.warning("Object of type %s cannot be JSON serialized. Skipping..." % type(argument))

        return result

    def to_sql(self) -> ORMCode:
        return ORMCode(self.function.__name__)

    def insert(self, session: sqlalchemy.orm.session.Session) -> ORMCode:
        code = self.to_sql()

        # set foreign key to designator if present
        if self.designator:
            designator = self.designator.insert(session)
            code.designator = designator.id

        session.add(code)
        session.commit()
        return code


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

    @property
    def name(self):
        return str(self)

    def to_json(self):
        return {"code": self.code.to_json(),
                "status": self.status.name,
                "start_time": self.start_time.isoformat() if self.start_time else None,
                "end_time": self.end_time.isoformat() if self.end_time else None,
                "id": id(self),
                "parent_id": id(self.parent) if self.parent else None
                }

    def __str__(self):
        return "Code: %s \n " \
               "start_time: %s \n " \
               "Status: %s \n " \
               "end_time: %s \n " \
               "" % (str(self.code), self.start_time, self.status, self.end_time)

    def __repr__(self):
        return str(self.code)

    def __len__(self):
        """Get the number of nodes that are in this subtree."""
        return 1 + sum([len(child) for child in self.children])

    def to_sql(self) -> ORMTaskTreeNode:
        """Convert this object to the corresponding object in the pycram.orm package.

        :returns:  corresponding pycram.orm.task.TaskTreeNode object
        """
        return ORMTaskTreeNode(None, self.start_time, self.end_time, self.status.name,
                               id(self.parent) if self.parent else None)

    def insert(self, session: sqlalchemy.orm.session.Session, recursive: bool = True,
               parent_id: Optional[int] = None) -> ORMTaskTreeNode:
        """
        Insert this node into the database.

        :param session: The current session with the database.
        :param recursive: Rather if the entire tree should be inserted or just this node, defaults to True
        :param parent_id: The primary key of the parent node, defaults to None

        :return: The ORM object that got inserted
        """

        # insert code
        code = self.code.insert(session)

        # convert self to orm object
        node = self.to_sql()
        node.code = code.id

        # set parent to id from constructor
        node.parent = parent_id

        # add the node to database to retrieve the new id
        session.add(node)
        session.commit()

        # if recursive, insert all children
        if recursive:
            [child.insert(session, ) for child in self.children]

        return node


class SimulatedTaskTree:
    """TaskTree for execution in a 'new' simulation."""

    def __enter__(self):
        """At the beginning of a with statement the current task tree and bullet world will be suspended and remembered.
        Fresh structures are then available inside the with statement."""
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
        """
        Restore the old state at the end of a with block.
        """
        global task_tree
        task_tree = self.suspended_tree
        BulletWorld.current_bullet_world.restore_state(self.world_state, self.objects2attached)
        pybullet.removeAllUserDebugItems()


task_tree: Optional[TaskTreeNode] = None
"""Current TaskTreeNode"""


def reset_tree() -> None:
    """
    Reset the current task tree to an empty root (NoOperation) node.
    """
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

        # get the "self" object:
        self_ = inspect.currentframe().f_back.f_locals.get("self")

        # if it is an action designator, save it for logging
        # TODO check if this is correct
        # if not self_.__name__ == "pycram.designators.action_designator.ActionDesignatorDescription":
        #   self_ = None
        self_ = None

        # create the code object that gets executed
        code = Code(fun, inspect.getcallargs(fun, *args, **kwargs), self_)

        task_tree = TaskTreeNode(code, parent=task_tree)

        # Try to execute the task
        try:
            task_tree.status = TaskStatus.CREATED
            task_tree.start_time = datetime.datetime.now()
            task_tree.code.execute()

            # if it succeeded set the flag
            task_tree.status = TaskStatus.SUCCEEDED

        # iff a PlanFailure occurs
        except PlanFailure as e:

            # log the error and set the flag
            logging.exception("Task execution failed at %s. Reason %s" % (str(task_tree.code), e))
            task_tree.status = TaskStatus.FAILED

        # set and time and update current node pointer
        task_tree.end_time = datetime.datetime.now()
        task_tree = task_tree.parent

    return handle_tree
