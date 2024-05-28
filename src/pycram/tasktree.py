"""Implementation of TaskTrees using anytree."""

# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from typing_extensions import TYPE_CHECKING

import datetime
import inspect
import logging
from typing_extensions import List, Optional, Callable

import anytree
import sqlalchemy.orm.session
import tqdm

from .datastructures.world import World
from .orm.tasktree import TaskTreeNode as ORMTaskTreeNode
from .orm.base import ProcessMetaData
from .plan_failures import PlanFailure
from .datastructures.enums import TaskStatus
from .datastructures.dataclasses import Color

if TYPE_CHECKING:
    from .designators.performables import Action


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

    """
    The reason why this task failed, optional
    """

    def __init__(self, action: Optional[Action] = NoOperation(), parent: Optional[TaskTreeNode] = None,
                 children: Optional[List[TaskTreeNode]] = None, reason: Optional[Exception] = None):
        """
        Create a TaskTreeNode

        :param action: The action and that is performed, defaults to None
        :param parent: The parent function of this function. None if this the parent, optional
        :param children: An iterable of TaskTreeNode with the ordered children, optional
        """
        super().__init__()

        if action is None:
            action = NoOperation()

        self.action = action
        self.status = TaskStatus.CREATED
        self.start_time = None
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
               "" % (str(self.action), self.start_time, self.status, self.end_time)

    def __repr__(self):
        return str(self.action.__class__.__name__)

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

        return ORMTaskTreeNode(self.start_time, self.end_time, self.status.name, reason)

    def insert(self, session: sqlalchemy.orm.session.Session, recursive: bool = True,
               parent: Optional[TaskTreeNode] = None, use_progress_bar: bool = True,
               progress_bar: Optional[tqdm.tqdm] = None) -> ORMTaskTreeNode:
        """
        Insert this node into the database.

        :param session: The current session with the database.
        :param recursive: Rather if the entire tree should be inserted or just this node, defaults to True
        :param parent: The parent node, defaults to None
        :param use_progress_bar: Rather to use a progressbar or not
        :param progress_bar: The progressbar to update. If a progress bar is desired and this is None, a new one will be
            created.

        :return: The ORM object that got inserted
        """
        if use_progress_bar:
            if not progress_bar:
                progress_bar = tqdm.tqdm(desc="Inserting TaskTree into database", leave=True, position=0,
                                         total=len(self) if recursive else 1)

        # convert self to orm object
        node = self.to_sql()

        # insert action if possible
        if getattr(self.action, "insert", None):
            action = self.action.insert(session)
            node.action = action
        else:
            action = None
            node.action = None

        # get and set metadata
        metadata = ProcessMetaData().insert(session)
        node.process_metadata = metadata

        # set node parent
        node.parent = parent

        # add the node to the session; note that the instance is not yet committed to the db, but rather in a
        # pending state
        session.add(node)

        if progress_bar:
            progress_bar.update()

        # if recursive, insert all children
        if recursive:
            [child.insert(session, parent=node, use_progress_bar=use_progress_bar, progress_bar=progress_bar)
             for child in self.children]

        # once recursion is done and the root node is reached again, commit the session to the database
        if self.parent is None:
            session.commit()

        return node


class SimulatedTaskTree:
    """TaskTree for execution in a 'new' simulation."""

    def __enter__(self):
        """At the beginning of a with statement the current task tree and world will be suspended and remembered.
        Fresh structures are then available inside the with statement."""
        global task_tree

        self.suspended_tree = task_tree
        self.world_state = World.current_world.save_state()
        self.simulated_root = TaskTreeNode()
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


task_tree: Optional[TaskTreeNode] = None
"""Current TaskTreeNode"""


def reset_tree() -> None:
    """
    Reset the current task tree to an empty root (NoOperation) node.
    """
    global task_tree
    task_tree = TaskTreeNode()
    task_tree.start_time = datetime.datetime.now()
    task_tree.status = TaskStatus.RUNNING


reset_tree()


def with_tree(fun: Callable) -> Callable:
    """
    Decorator that records the function name, arguments and execution metadata in the task tree.

    :param fun: The function to record the data from.
    """

    def handle_tree(*args, **kwargs):

        # get the task tree
        global task_tree

        # parse keyword arguments
        keyword_arguments = inspect.getcallargs(fun, *args, **kwargs)

        # try to get self object since this represents the action object
        action = keyword_arguments.get("self", None)

        # create the task tree node
        task_tree = TaskTreeNode(action, parent=task_tree)

        # Try to execute the task
        try:
            task_tree.status = TaskStatus.CREATED
            task_tree.start_time = datetime.datetime.now()
            result = fun(*args, **kwargs)

            # if it succeeded set the flag
            task_tree.status = TaskStatus.SUCCEEDED

        # iff a PlanFailure occurs
        except PlanFailure as e:

            # log the error and set the flag
            logging.exception("Task execution failed at %s. Reason %s" % (repr(task_tree), e))
            task_tree.reason = e
            task_tree.status = TaskStatus.FAILED
            raise e
        finally:
            # set and time and update current node pointer
            task_tree.end_time = datetime.datetime.now()
            task_tree = task_tree.parent
        return result

    return handle_tree
