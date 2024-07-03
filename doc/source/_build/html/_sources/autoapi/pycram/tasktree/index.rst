:py:mod:`pycram.tasktree`
=========================

.. py:module:: pycram.tasktree

.. autoapi-nested-parse::

   Implementation of TaskTrees using anytree.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.tasktree.NoOperation
   pycram.tasktree.TaskTreeNode
   pycram.tasktree.SimulatedTaskTree



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.tasktree.reset_tree
   pycram.tasktree.with_tree



Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.tasktree.task_tree


.. py:class:: NoOperation


   .. py:method:: perform()


   .. py:method:: __repr__()

      Return repr(self).



.. py:class:: TaskTreeNode(action: typing_extensions.Optional[pycram.designators.performables.Action] = NoOperation(), parent: typing_extensions.Optional[TaskTreeNode] = None, children: typing_extensions.Optional[typing_extensions.List[TaskTreeNode]] = None, reason: typing_extensions.Optional[Exception] = None)


   Bases: :py:obj:`anytree.NodeMixin`

   TaskTreeNode represents one function that was called during a pycram plan.
   Additionally, meta information is stored.

   Create a TaskTreeNode

   :param action: The action and that is performed, defaults to None
   :param parent: The parent function of this function. None if this the parent, optional
   :param children: An iterable of TaskTreeNode with the ordered children, optional

   .. py:property:: name


   .. py:attribute:: action
      :type: typing_extensions.Optional[pycram.designators.performables.Action]

      The action and that is performed or None if nothing was performed

   .. py:attribute:: status
      :type: pycram.datastructures.enums.TaskStatus

      The status of the node from the TaskStatus enum.

   .. py:attribute:: start_time
      :type: typing_extensions.Optional[datetime.datetime]

      The starting time of the function, optional

   .. py:attribute:: end_time
      :type: typing_extensions.Optional[datetime.datetime]

      The ending time of the function, optional

   .. py:method:: __str__()

      Return str(self).


   .. py:method:: __repr__()

      Return repr(self).


   .. py:method:: __len__()

      Get the number of nodes that are in this subtree.


   .. py:method:: to_sql() -> pycram.orm.tasktree.TaskTreeNode

      Convert this object to the corresponding object in the pycram.orm package.

      :returns:  corresponding pycram.orm.task.TaskTreeNode object


   .. py:method:: insert(session: sqlalchemy.orm.session.Session, recursive: bool = True, parent: typing_extensions.Optional[TaskTreeNode] = None, use_progress_bar: bool = True, progress_bar: typing_extensions.Optional[tqdm.tqdm] = None) -> pycram.orm.tasktree.TaskTreeNode

      Insert this node into the database.

      :param session: The current session with the database.
      :param recursive: Rather if the entire tree should be inserted or just this node, defaults to True
      :param parent: The parent node, defaults to None
      :param use_progress_bar: Rather to use a progressbar or not
      :param progress_bar: The progressbar to update. If a progress bar is desired and this is None, a new one will be
          created.

      :return: The ORM object that got inserted



.. py:class:: SimulatedTaskTree


   TaskTree for execution in a 'new' simulation.

   .. py:method:: __enter__()

      At the beginning of a with statement the current task tree and world will be suspended and remembered.
      Fresh structures are then available inside the with statement.


   .. py:method:: __exit__(exc_type, exc_val, exc_tb)

      Restore the old state at the end of a with block.



.. py:data:: task_tree
   :type: typing_extensions.Optional[TaskTreeNode]

   Current TaskTreeNode

.. py:function:: reset_tree() -> None

   Reset the current task tree to an empty root (NoOperation) node.


.. py:function:: with_tree(fun: typing_extensions.Callable) -> typing_extensions.Callable

   Decorator that records the function name, arguments and execution metadata in the task tree.

   :param fun: The function to record the data from.


