"""Implementation of TaskTrees using anytree."""

# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import datetime
import inspect
import json
import logging
from typing import List, Dict, Optional, Callable, Any

import anytree
import pybullet
import sqlalchemy.orm.session
import tqdm

from .bullet_world import BulletWorld
from .orm.task import (Code as ORMCode, TaskTreeNode as ORMTaskTreeNode)
from .orm.base import ProcessMetaData
from .plan_failures import PlanFailure
from .language import Code
from .enums import TaskStatus


class TaskCode(Code):
    def __str__(self) -> str:
        if "self" in self.kwargs:
            class_name = self.kwargs["self"].__class__.__name__
        else:
            class_name = ""
        function_name = self.function.__name__

        return f"{function_name}({class_name})"

    def __eq__(self, other):
        return isinstance(other, Code) and other.function.__name__ == self.function.__name__ \
            and other.kwargs == self.kwargs

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
        self_ = self.kwargs.get("self")

        if self_ and getattr(self_, "insert", None):
            designator = self_.insert(session)
            code.designator_id = designator.id

        # get and set metadata
        metadata = ProcessMetaData().insert(session)
        code.process_metadata_id = metadata.id

        session.add(code)
        session.commit()
        return code


class NoOperation(TaskCode):
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
    :ivar reason: The reason why this task failed, optional
    """

    def __init__(self, code: TaskCode = NoOperation(), parent: Optional[TaskTreeNode] = None,
                 children: Optional[List[TaskTreeNode]] = None, reason: Optional[Exception] = None):
        """
        Create a TaskTreeNode

        :param code: The function and its arguments that got called as TaskCode object, defaults to NoOperation()
        :param parent: The parent function of this function. None if this the parent, optional
        :param children: An iterable of TaskTreeNode with the ordered children, optional
        """
        super().__init__()
        self.code: TaskCode = code
        self.status: TaskStatus = TaskStatus.CREATED
        self.start_time: Optional[datetime.datetime] = None
        self.end_time: Optional[datetime.datetime] = None
        self.parent = parent
        self.reason: Optional[Exception] = reason

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

        if self.reason:
            reason = type(self.reason).__name__
        else:
            reason = None

        return ORMTaskTreeNode(None, self.start_time, self.end_time, self.status.name,
                               reason, id(self.parent) if self.parent else None)

    def insert(self, session: sqlalchemy.orm.session.Session, recursive: bool = True,
               parent_id: Optional[int] = None, use_progress_bar: bool = True,
               progress_bar: Optional[tqdm.tqdm] = None) -> ORMTaskTreeNode:
        """
        Insert this node into the database.

        :param session: The current session with the database.
        :param recursive: Rather if the entire tree should be inserted or just this node, defaults to True
        :param parent_id: The primary key of the parent node, defaults to None
        :param use_progress_bar: Rather to use a progressbar or not
        :param progress_bar: The progressbar to update. If a progress bar is desired and this is None, a new one will be
            created.

        :return: The ORM object that got inserted
        """
        if use_progress_bar:
            if not progress_bar:
                progress_bar = tqdm.tqdm(desc="Inserting TaskTree into database", leave=True, position=0,
                                         total=len(self) if recursive else 1)

        # insert code
        code = self.code.insert(session)

        # convert self to orm object
        node = self.to_sql()
        node.code_id = code.id

        # get and set metadata
        metadata = ProcessMetaData().insert(session)
        node.process_metadata_id = metadata.id

        # set parent to id from constructor
        node.parent_id = parent_id

        # add the node to database to retrieve the new id
        session.add(node)
        session.commit()

        if progress_bar:
            progress_bar.update()

        # if recursive, insert all children
        if recursive:
            [child.insert(session, parent_id=node.id, use_progress_bar=use_progress_bar, progress_bar=progress_bar)
             for child in self.children]

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
        self.simulated_root = TaskTreeNode(code=TaskCode(simulation))
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

        # create the code object that gets executed
        code = TaskCode(fun, inspect.getcallargs(fun, *args, **kwargs))

        task_tree = TaskTreeNode(code, parent=task_tree)

        # Try to execute the task
        try:
            task_tree.status = TaskStatus.CREATED
            task_tree.start_time = datetime.datetime.now()
            result = task_tree.code.execute()

            # if it succeeded set the flag
            task_tree.status = TaskStatus.SUCCEEDED

        # iff a PlanFailure occurs
        except PlanFailure as e:

            # log the error and set the flag
            logging.exception("Task execution failed at %s. Reason %s" % (str(task_tree.code), e))
            task_tree.reason = e
            task_tree.status = TaskStatus.FAILED
            raise e
        finally:
            # set and time and update current node pointer
            task_tree.end_time = datetime.datetime.now()
            task_tree = task_tree.parent
        return result

    return handle_tree
