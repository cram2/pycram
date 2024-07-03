:py:mod:`pycram.language`
=========================

.. py:module:: pycram.language


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.language.Language
   pycram.language.Repeat
   pycram.language.Monitor
   pycram.language.Sequential
   pycram.language.TryInOrder
   pycram.language.Parallel
   pycram.language.TryAll
   pycram.language.Code




.. py:class:: Language(parent: anytree.NodeMixin = None, children: typing_extensions.Iterable[anytree.NodeMixin] = None)


   Bases: :py:obj:`anytree.NodeMixin`

   Parent class for language expressions. Implements the operators as well as methods to reduce the resulting language
   tree.

   Default constructor for anytree nodes. If the parent is none this is the root node.

   :param parent: The parent node of this node
   :param children: All children of this node as a tuple oder iterable

   .. py:attribute:: parallel_blocklist
      :value: ['PickUpAction', 'PlaceAction', 'OpenAction', 'CloseAction', 'TransportAction', 'GraspingAction']

      

   .. py:attribute:: do_not_use_giskard
      :value: ['SetGripperAction', 'MoveGripperMotion', 'DetectAction', 'DetectingMotion']

      

   .. py:attribute:: block_list
      :type: typing_extensions.List[int]
      :value: []

      List of thread ids which should be blocked from execution.

   .. py:method:: resolve() -> Language

      Dummy method for compatability to designator descriptions

      :return: self reference


   .. py:method:: perform()
      :abstractmethod:

      This method should be overwritten in subclasses and implement the behaviour of the language expression regarding
      each child.


   .. py:method:: __add__(other: Language) -> Sequential

      Language expression for sequential execution.

      :param other: Another Language expression, either a designator or language expression
      :return: A :func:`~Sequential` object which is the new root node of the language tree


   .. py:method:: __sub__(other: Language) -> TryInOrder

      Language expression for try in order.

      :param other: Another Language expression, either a designator or language expression
      :return: A :func:`~TryInOrder` object which is the new root node of the language tree


   .. py:method:: __or__(other: Language) -> Parallel

      Language expression for parallel execution.

      :param other: Another Language expression, either a designator or language expression
      :return: A :func:`~Parallel` object which is the new root node of the language tree


   .. py:method:: __xor__(other: Language) -> TryAll

      Language expression for try all execution.

      :param other: Another Language expression, either a designator or language expression
      :return: A :func:`~TryAll` object which is the new root node of the language tree


   .. py:method:: __rshift__(other: Language)

      Operator for Monitors, this always makes the Monitor the parent of the other expression.

      :param other: Another Language expression
      :return: The Monitor which is now the new root node.


   .. py:method:: __mul__(other: int)

      Language expression for Repeated execution. The other attribute of this operator has to be an integer.

      :param other: An integer which states how often the Language expression should be repeated
      :return: A :func:`~Repeat` object which is the new root node of the language tree


   .. py:method:: __rmul__(other: int)

      Language expression for Repeated execution. The other attribute of this operator has to be an integer. This is
      the reversed operator of __mul__ which allows to write:

      .. code-block:: python

          2 * ParkAction()

      :param other: An integer which states how often the Language expression should be repeated
      :return: A :func:`~Repeat` object which is the new root node of the language tree


   .. py:method:: simplify() -> Language

      Simplifies the language tree by merging which have a parent-child relation and are of the same type.

      .. code-block::

          <pycram.new_language.Parallel>
          ├── <pycram.new_language.Parallel>
          │   ├── <pycram.designators.action_designator.NavigateAction>
          │   └── <pycram.designators.action_designator.MoveTorsoAction>
          └── <pycram.designators.action_designator.DetectAction>

          would be simplified to:

         <pycram.new_language.Parallel>
          ├── <pycram.designators.action_designator.NavigateAction>
          ├── <pycram.designators.action_designator.MoveTorsoAction>
          └── <pycram.designators.action_designator.DetectAction>



   .. py:method:: merge_nodes(node1: anytree.Node, node2: anytree.Node) -> None
      :staticmethod:

      Merges node1 with node2 in a tree. The children of node2 will be re-parented to node1 and node2 will be deleted
      from the tree.

      :param node1: Node that should be left in the tree
      :param node2: Node which children should be appended to node1 and then deleted


   .. py:method:: interrupt() -> None
      :abstractmethod:

      Base method for interrupting the execution of Language expression. To be overwritten in a sub-class.



.. py:class:: Repeat(parent: anytree.NodeMixin = None, children: typing_extensions.Iterable[anytree.NodeMixin] = None, repeat: int = 1)


   Bases: :py:obj:`Language`

   Executes all children a given number of times.

   Initializes the Repeat expression with a parent and children for the language tree construction and a number
   which states how often the children should be executed.

   :param parent: Parent node of this node, if None this will be the root node
   :param children: A list of children of this node
   :param repeat: An integer of how often the children should be executed.

   .. py:method:: perform()

      Behaviour of repeat, executes all children in a loop as often as stated on initialization.

      :return:


   .. py:method:: interrupt() -> None

      Stops the execution of this language expression by setting the ``interrupted`` variable to True, adding this
      thread to the block_list in ProcessModule and interrupting the current giskard goal



.. py:class:: Monitor(condition: typing_extensions.Union[typing_extensions.Callable, pycram.fluent.Fluent] = None)


   Bases: :py:obj:`Language`

   Monitors a Language Expression and interrupts it when the given condition is evaluated to True.

   Behaviour:
       This Monitor is attached to a language expression, when perform on this Monitor is called it will start a new
       thread which continuously checks if the condition is True. When the condition is True the interrupt function of
       the child will be called.

   When initializing a Monitor a condition must be provided. The condition is a callable or a Fluent which returns         True or False.

   :param condition: The condition upon which the Monitor should interrupt the attached language expression.

   .. py:method:: perform()

      Behavior of the Monitor, starts a new Thread which checks the condition and then performs the attached language
      expression

      :return: The result of the attached language expression


   .. py:method:: interrupt() -> None

      Calls interrupt for each child



.. py:class:: Sequential(parent: anytree.NodeMixin = None, children: typing_extensions.Iterable[anytree.NodeMixin] = None)


   Bases: :py:obj:`Language`

   Executes all children sequentially, an exception while executing a child does not terminate the whole process.
   Instead, the exception is saved to a list of all exceptions thrown during execution and returned.

   Behaviour:
       Return the state :py:attr:`~State.SUCCEEDED` *iff* all children are executed without exception.
       In any other case the State :py:attr:`~State.FAILED` will be returned.

   Default constructor for anytree nodes. If the parent is none this is the root node.

   :param parent: The parent node of this node
   :param children: All children of this node as a tuple oder iterable

   .. py:method:: perform() -> pycram.datastructures.enums.State

      Behaviour of Sequential, calls perform() on each child sequentially

      :return: The state according to the behaviour described in :func:`Sequential`


   .. py:method:: interrupt() -> None

      Interrupts the execution of this language expression by setting the ``interrupted`` variable to True and calling
      interrupt on the current giskard goal.



.. py:class:: TryInOrder(parent: anytree.NodeMixin = None, children: typing_extensions.Iterable[anytree.NodeMixin] = None)


   Bases: :py:obj:`Language`

   Executes all children sequentially, an exception while executing a child does not terminate the whole process.
   Instead, the exception is saved to a list of all exceptions thrown during execution and returned.

   Behaviour:
       Returns the State :py:attr:`~State.SUCCEEDED` if one or more children are executed without
       exception. In the case that all children could not be executed the State :py:attr:`~State.FAILED` will be returned.

   Default constructor for anytree nodes. If the parent is none this is the root node.

   :param parent: The parent node of this node
   :param children: All children of this node as a tuple oder iterable

   .. py:method:: perform() -> pycram.datastructures.enums.State

      Behaviour of TryInOrder, calls perform() on each child sequentially and catches raised exceptions.

      :return: The state according to the behaviour described in :func:`TryInOrder`


   .. py:method:: interrupt() -> None

      Interrupts the execution of this language expression by setting the ``interrupted`` variable to True, adding
      the current thread to the block_list in Language and interrupting the current giskard goal.



.. py:class:: Parallel(parent: anytree.NodeMixin = None, children: typing_extensions.Iterable[anytree.NodeMixin] = None)


   Bases: :py:obj:`Language`

   Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
   exceptions during execution will be caught, saved to a list and returned upon end.

   Behaviour:
       Returns the State :py:attr:`~State.SUCCEEDED` *iff* all children could be executed without an exception. In any
       other case the State :py:attr:`~State.FAILED` will be returned.

   Default constructor for anytree nodes. If the parent is none this is the root node.

   :param parent: The parent node of this node
   :param children: All children of this node as a tuple oder iterable

   .. py:method:: perform() -> pycram.datastructures.enums.State

      Behaviour of Parallel, creates a new thread for each child and calls perform() of the child in the respective
      thread.

      :return: The state according to the behaviour described in :func:`Parallel`


   .. py:method:: interrupt() -> None

      Interrupts the execution of this language expression by setting the ``interrupted`` variable to True, adding the
      thread id of all parallel execution threads to the block_list in Language and interrupting the current giskard
      goal.



.. py:class:: TryAll(parent: anytree.NodeMixin = None, children: typing_extensions.Iterable[anytree.NodeMixin] = None)


   Bases: :py:obj:`Language`

   Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
   exceptions during execution will be caught, saved to a list and returned upon end.

   Behaviour:
       Returns the State :py:attr:`~State.SUCCEEDED` if one or more children could be executed without raising an
       exception. If all children fail the State :py:attr:`~State.FAILED` will be returned.

   Default constructor for anytree nodes. If the parent is none this is the root node.

   :param parent: The parent node of this node
   :param children: All children of this node as a tuple oder iterable

   .. py:method:: perform() -> pycram.datastructures.enums.State

      Behaviour of TryAll, creates a new thread for each child and executes all children in their respective threads.

      :return: The state according to the behaviour described in :func:`TryAll`


   .. py:method:: interrupt() -> None

      Interrupts the execution of this language expression by setting the ``interrupted`` variable to True, adding the
      thread id of all parallel execution threads to the block_list in Language and interrupting the current giskard



.. py:class:: Code(function: typing_extensions.Optional[typing_extensions.Callable] = None, kwargs: typing_extensions.Optional[typing_extensions.Dict] = None)


   Bases: :py:obj:`Language`

   Executable code block in a plan.

   :ivar function: The function (plan) that was called
   :ivar kwargs: Dictionary holding the keyword arguments of the function

   Initialize a code call

   :param function: The function that was called
   :param kwargs: The keyword arguments of the function as dict

   .. py:method:: execute() -> typing_extensions.Any

      Execute the code with its arguments

      :returns: Anything that the function associated with this object will return.


   .. py:method:: interrupt() -> None
      :abstractmethod:

      Base method for interrupting the execution of Language expression. To be overwritten in a sub-class.



