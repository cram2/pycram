# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import time
from typing_extensions import Iterable, Optional, Callable, Dict, Any, List, Union
from anytree import NodeMixin, Node, PreOrderIter

from pycram.datastructures.enums import State
import threading

from .fluent import Fluent
from .plan_failures import PlanFailure, NotALanguageExpression
from .external_interfaces import giskard


class Language(NodeMixin):
    """
    Parent class for language expressions. Implements the operators as well as methods to reduce the resulting language
    tree.
    """
    parallel_blocklist = ["PickUpAction", "PlaceAction", "OpenAction", "CloseAction", "TransportAction", "GraspingAction"]
    do_not_use_giskard = ["SetGripperAction", "MoveGripperMotion", "DetectAction", "DetectingMotion"]
    block_list: List[int] = []
    """List of thread ids which should be blocked from execution."""

    def __init__(self, parent: NodeMixin = None, children: Iterable[NodeMixin] = None):
        """
        Default constructor for anytree nodes. If the parent is none this is the root node.

        :param parent: The parent node of this node
        :param children: All children of this node as a tuple oder iterable
        """
        self.parent = parent
        self.exceptions = {}
        self.state = None
        self.executing_thread = {}
        self.threads: List[threading.Thread] = []
        self.interrupted = False
        self.name = self.__class__.__name__
        if children:
            self.children: Language = children

    def resolve(self) -> Language:
        """
        Dummy method for compatability to designator descriptions

        :return: self reference
        """
        return self

    def perform(self):
        """
        This method should be overwritten in subclasses and implement the behaviour of the language expression regarding
        each child.
        """
        raise NotImplementedError

    def __add__(self, other: Language) -> Sequential:
        """
        Language expression for sequential execution.

        :param other: Another Language expression, either a designator or language expression
        :return: A :func:`~Sequential` object which is the new root node of the language tree
        """
        if not issubclass(other.__class__, Language):
            raise NotALanguageExpression(
                f"Only classes that inherit from the Language class can be used with the plan language, these are usually Designators or Code objects. \nThe object '{other}' does not inherit from the Language class.")
        return Sequential(parent=None, children=(self, other)).simplify()

    def __sub__(self, other: Language) -> TryInOrder:
        """
        Language expression for try in order.

        :param other: Another Language expression, either a designator or language expression
        :return: A :func:`~TryInOrder` object which is the new root node of the language tree
        """
        if not issubclass(other.__class__, Language):
            raise NotALanguageExpression(
                f"Only classes that inherit from the Language class can be used with the plan language, these are usually Designators or Code objects. \nThe object '{other}' does not inherit from the Language class.")
        return TryInOrder(parent=None, children=(self, other)).simplify()

    def __or__(self, other: Language) -> Parallel:
        """
        Language expression for parallel execution.

        :param other: Another Language expression, either a designator or language expression
        :return: A :func:`~Parallel` object which is the new root node of the language tree
        """
        if not issubclass(other.__class__, Language):
            raise NotALanguageExpression(
                f"Only classes that inherit from the Language class can be used with the plan language, these are usually Designators or Code objects. \nThe object '{other}' does not inherit from the Language class.")
        if self.__class__.__name__ in self.parallel_blocklist or other.__class__.__name__ in self.parallel_blocklist:
            raise AttributeError(
                f"You can not execute the Designator {self if self.__class__.__name__ in self.parallel_blocklist else other} in a parallel language expression.")

        return Parallel(parent=None, children=(self, other)).simplify()

    def __xor__(self, other: Language) -> TryAll:
        """
        Language expression for try all execution.

        :param other: Another Language expression, either a designator or language expression
        :return: A :func:`~TryAll` object which is the new root node of the language tree
        """
        if not issubclass(other.__class__, Language):
            raise NotALanguageExpression(
                f"Only classes that inherit from the Language class can be used with the plan language, these are usually Designators or Code objects. \nThe object '{other}' does not inherit from the Language class.")
        if self.__class__.__name__ in self.parallel_blocklist or other.__class__.__name__ in self.parallel_blocklist:
            raise AttributeError(
                f"You can not execute the Designator {self if self.__class__.__name__ in self.parallel_blocklist else other} in a try all language expression.")
        return TryAll(parent=None, children=(self, other)).simplify()

    def __rshift__(self, other: Language):
        """
        Operator for Monitors, this always makes the Monitor the parent of the other expression.
        
        :param other: Another Language expression
        :return: The Monitor which is now the new root node.
        """
        if isinstance(self, Monitor) and isinstance(other, Monitor):
            raise AttributeError("You can't attach a Monitor to another Monitor.")
        if isinstance(self, Monitor):
            self.children = [other]
            return self
        elif isinstance(other, Monitor):
            other.children = [self]
            return other

    def __mul__(self, other: int):
        """
        Language expression for Repeated execution. The other attribute of this operator has to be an integer.

        :param other: An integer which states how often the Language expression should be repeated
        :return: A :func:`~Repeat` object which is the new root node of the language tree
        """
        if not isinstance(other, int):
            raise AttributeError("Repeat can only be used in combination with integers")
        return Repeat(parent=None, children=[self], repeat=other)

    def __rmul__(self, other: int):
        """
        Language expression for Repeated execution. The other attribute of this operator has to be an integer. This is
        the reversed operator of __mul__ which allows to write:

        .. code-block:: python
        
            2 * ParkAction()

        :param other: An integer which states how often the Language expression should be repeated
        :return: A :func:`~Repeat` object which is the new root node of the language tree
        """
        if not isinstance(other, int):
            raise AttributeError("Repeat can only be used in combination with integers")
        return Repeat(parent=None, children=[self], repeat=other)

    def simplify(self) -> Language:
        """
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

        """
        for node in PreOrderIter(self.root):
            for child in node.children:
                if isinstance(child, Monitor):
                    continue
                if type(node) is type(child):
                    self.merge_nodes(node, child)
        return self.root

    @staticmethod
    def merge_nodes(node1: Node, node2: Node) -> None:
        """
        Merges node1 with node2 in a tree. The children of node2 will be re-parented to node1 and node2 will be deleted
        from the tree.

        :param node1: Node that should be left in the tree
        :param node2: Node which children should be appended to node1 and then deleted
        """
        node2.parent = None
        node1.children = node2.children + node1.children

    def interrupt(self) -> None:
        """
        Base method for interrupting the execution of Language expression. To be overwritten in a sub-class.
        """
        raise NotImplementedError


class Repeat(Language):
    """
    Executes all children a given number of times.
    """
    def perform(self):
        """
        Behaviour of repeat, executes all children in a loop as often as stated on initialization.

        :return:
        """
        for i in range(self.repeat):
            for child in self.children:
                if self.interrupted:
                    return
                try:
                    child.resolve().perform()
                except PlanFailure as e:
                    self.root.exceptions[self] = e

    def __init__(self, parent: NodeMixin = None, children: Iterable[NodeMixin] = None, repeat: int = 1):
        """
        Initializes the Repeat expression with a parent and children for the language tree construction and a number
        which states how often the children should be executed.

        :param parent: Parent node of this node, if None this will be the root node
        :param children: A list of children of this node
        :param repeat: An integer of how often the children should be executed.
        """
        super().__init__(parent, children)
        self.repeat: int = repeat

    def interrupt(self) -> None:
        """
        Stops the execution of this language expression by setting the ``interrupted`` variable to True, adding this
        thread to the block_list in ProcessModule and interrupting the current giskard goal
        """
        self.interrupted = True
        self.block_list.append(threading.get_ident())
        if giskard.giskard_wrapper:
            giskard.giskard_wrapper.interrupt()


class Monitor(Language):
    """
    Monitors a Language Expression and interrupts it when the given condition is evaluated to True.

    Behaviour:
        This Monitor is attached to a language expression, when perform on this Monitor is called it will start a new
        thread which continuously checks if the condition is True. When the condition is True the interrupt function of
        the child will be called.
    """
    def __init__(self, condition: Union[Callable, Fluent] = None):
        """
        When initializing a Monitor a condition must be provided. The condition is a callable or a Fluent which returns \
        True or False.

        :param condition: The condition upon which the Monitor should interrupt the attached language expression.
        """
        super().__init__(None, None)
        self.kill_event = threading.Event()
        if callable(condition):
            self.condition = Fluent(condition)
        elif isinstance(condition, Fluent):
            self.condition = condition
        else:
            raise AttributeError("The condition of a Monitor has to be a Callable or a Fluent")

    def perform(self):
        """
        Behavior of the Monitor, starts a new Thread which checks the condition and then performs the attached language
        expression

        :return: The result of the attached language expression
        """
        def check_condition():
            while not self.condition.get_value() and not self.kill_event.is_set():
                time.sleep(0.1)
            if self.kill_event.is_set():
                return
            for child in self.children:
                child.interrupt()

        t = threading.Thread(target=check_condition)
        t.start()
        res = self.children[0].perform()
        self.kill_event.set()
        t.join()
        return res

    def interrupt(self) -> None:
        """
        Calls interrupt for each child
        """
        for child in self.children:
            child.interrupt()


class Sequential(Language):
    """
    Executes all children sequentially, an exception while executing a child does not terminate the whole process.
    Instead, the exception is saved to a list of all exceptions thrown during execution and returned.

    Behaviour:
        Return the state :py:attr:`~State.SUCCEEDED` *iff* all children are executed without exception.
        In any other case the State :py:attr:`~State.FAILED` will be returned.
    """

    def perform(self) -> State:
        """
        Behaviour of Sequential, calls perform() on each child sequentially

        :return: The state according to the behaviour described in :func:`Sequential`
        """
        try:
            for child in self.children:
                if self.interrupted:
                    if threading.get_ident() in self.block_list:
                        self.block_list.remove(threading.get_ident())
                    return
                self.root.executing_thread[child] = threading.get_ident()
                child.resolve().perform()
        except PlanFailure as e:
            self.root.exceptions[self] = e
            return State.FAILED
        return State.SUCCEEDED

    def interrupt(self) -> None:
        """
        Interrupts the execution of this language expression by setting the ``interrupted`` variable to True and calling
        interrupt on the current giskard goal.
        """
        self.interrupted = True
        self.block_list.append(threading.get_ident())
        if giskard.giskard_wrapper:
            giskard.giskard_wrapper.interrupt()


class TryInOrder(Language):
    """
    Executes all children sequentially, an exception while executing a child does not terminate the whole process.
    Instead, the exception is saved to a list of all exceptions thrown during execution and returned.

    Behaviour:
        Returns the State :py:attr:`~State.SUCCEEDED` if one or more children are executed without
        exception. In the case that all children could not be executed the State :py:attr:`~State.FAILED` will be returned.
    """

    def perform(self) -> State:
        """
        Behaviour of TryInOrder, calls perform() on each child sequentially and catches raised exceptions.

        :return: The state according to the behaviour described in :func:`TryInOrder`
        """
        failure_list = []
        for child in self.children:
            if self.interrupted:
                if threading.get_ident() in self.block_list:
                    self.block_list.remove(threading.get_ident())
                return
            try:
                child.resolve().perform()
            except PlanFailure as e:
                failure_list.append(e)
        if len(failure_list) > 0:
            self.root.exceptions[self] = failure_list
        if len(failure_list) == len(self.children):
            self.root.exceptions[self] = failure_list
            return State.FAILED
        else:
            return State.SUCCEEDED

    def interrupt(self) -> None:
        """
        Interrupts the execution of this language expression by setting the ``interrupted`` variable to True, adding
        the current thread to the block_list in Language and interrupting the current giskard goal.
        """
        self.interrupted = True
        self.block_list.append(threading.get_ident())
        if giskard.giskard_wrapper:
            giskard.giskard_wrapper.interrupt()


class Parallel(Language):
    """
    Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
    exceptions during execution will be caught, saved to a list and returned upon end.

    Behaviour:
        Returns the State :py:attr:`~State.SUCCEEDED` *iff* all children could be executed without an exception. In any
        other case the State :py:attr:`~State.FAILED` will be returned.
    """

    def perform(self) -> State:
        """
        Behaviour of Parallel, creates a new thread for each child and calls perform() of the child in the respective
        thread.

        :return: The state according to the behaviour described in :func:`Parallel`
        """

        def lang_call(child_node):
            if ("DesignatorDescription" in [cls.__name__ for cls in child_node.__class__.__mro__]
                    and self.__class__.__name__ not in self.do_not_use_giskard):
                if self not in giskard.par_threads.keys():
                    giskard.par_threads[self] = [threading.get_ident()]
                else:
                    giskard.par_threads[self].append(threading.get_ident())
            try:
                self.root.executing_thread[child] = threading.get_ident()
                child_node.resolve().perform()
            except PlanFailure as e:
                if self in self.root.exceptions.keys():
                    self.root.exceptions[self].append(e)
                else:
                    self.root.exceptions[self] = [e]

        for child in self.children:
            if self.interrupted:
                break
            t = threading.Thread(target=lambda: lang_call(child))
            t.start()
            self.threads.append(t)
        for thread in self.threads:
            thread.join()
            if thread.ident in self.block_list:
                self.block_list.remove(thread.ident)
        if self in self.root.exceptions.keys() and len(self.root.exceptions[self]) != 0:
            return State.FAILED
        return State.SUCCEEDED

    def interrupt(self) -> None:
        """
        Interrupts the execution of this language expression by setting the ``interrupted`` variable to True, adding the
        thread id of all parallel execution threads to the block_list in Language and interrupting the current giskard
        goal.
        """
        self.interrupted = True
        self.block_list += [t.ident for t in self.threads]
        if giskard.giskard_wrapper:
            giskard.giskard_wrapper.interrupt()


class TryAll(Language):
    """
    Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
    exceptions during execution will be caught, saved to a list and returned upon end.

    Behaviour:
        Returns the State :py:attr:`~State.SUCCEEDED` if one or more children could be executed without raising an
        exception. If all children fail the State :py:attr:`~State.FAILED` will be returned.
    """

    def perform(self) -> State:
        """
        Behaviour of TryAll, creates a new thread for each child and executes all children in their respective threads.

        :return: The state according to the behaviour described in :func:`TryAll`
        """
        self.threads: List[threading.Thread] = []
        failure_list = []

        def lang_call(child_node):
            if ("DesignatorDescription" in [cls.__name__ for cls in child_node.__class__.__mro__]
                    and self.__class__.__name__ not in self.do_not_use_giskard):
                if self not in giskard.par_threads.keys():
                    giskard.par_threads[self] = [threading.get_ident()]
                else:
                    giskard.par_threads[self].append(threading.get_ident())
            try:
                child_node.resolve().perform()
            except PlanFailure as e:
                failure_list.append(e)
                if self in self.root.exceptions.keys():
                    self.root.exceptions[self].append(e)
                else:
                    self.root.exceptions[self] = [e]

        for child in self.children:
            t = threading.Thread(target=lambda: lang_call(child))
            t.start()
            self.threads.append(t)
        for thread in self.threads:
            thread.join()
            if thread.ident in self.block_list:
                self.block_list.remove(thread.ident)
        if len(self.children) == len(failure_list):
            self.root.exceptions[self] = failure_list
            return State.FAILED
        else:
            return State.SUCCEEDED

    def interrupt(self) -> None:
        """
        Interrupts the execution of this language expression by setting the ``interrupted`` variable to True, adding the
        thread id of all parallel execution threads to the block_list in Language and interrupting the current giskard
        """
        self.interrupted = True
        self.block_list += [t.ident for t in self.threads]
        if giskard.giskard_wrapper:
            giskard.giskard_wrapper.interrupt()


class Code(Language):
    """
    Executable code block in a plan.

    :ivar function: The function (plan) that was called
    :ivar kwargs: Dictionary holding the keyword arguments of the function
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
        self.kwargs: Dict[str, Any] = kwargs
        self.perform = self.execute

    def execute(self) -> Any:
        """
        Execute the code with its arguments

        :returns: Anything that the function associated with this object will return.
        """
        return self.function(**self.kwargs)

    def interrupt(self) -> None:
        raise NotImplementedError


