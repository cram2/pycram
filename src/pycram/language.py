# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from dataclasses import dataclass
from queue import Queue
from typing_extensions import Iterable, Optional, Callable, Dict, Any, List, Union, Tuple, Self, Sequence

from .datastructures.enums import State
import threading

from .fluent import Fluent
from .failures import PlanFailure, NotALanguageExpression
from .external_interfaces import giskard
from .ros import  sleep
from .plan import PlanNode, Plan


def add_language_plan(child_plans: Iterable[Plan], root_node: LanguageNode) -> Plan:
    lang_plan = Plan()
    lang_plan.add_edge(lang_plan.root, root_node)
    for plan in child_plans:
        lang_plan.mount(plan)

    plan = Plan.current_plan
    plan.mount(lang_plan, plan.current_node)
    return lang_plan


def sequential(*plans: Sequence[Plan]) -> Plan:
    return add_language_plan(plans, SequentialNode())

def parallel(*plans: Sequence[Plan]) -> Plan:
    return add_language_plan(plans, ParallelNode())

def try_in_order(*plans: Sequence[Plan]) -> Plan:
    return add_language_plan(plans, TryInOrderNode())

def try_all(*plans: Sequence[Plan]) -> Plan:
    return add_language_plan(plans, TryAllNode())

def repeat(*plans: Sequence[Plan], repeat=1) -> Plan:
    return add_language_plan(plans, RepeatNode(repeat=repeat))



class LanguageMixin:
    """
    Parent class for language expressions. Implements the operators as well as methods to reduce the resulting language
    tree.
    """
    parallel_blocklist = ["PickUpAction", "PlaceAction", "OpenAction", "CloseAction", "TransportAction", "GraspingAction"]
    do_not_use_giskard = ["SetGripperAction", "MoveGripperMotion", "DetectAction", "DetectingMotion"]
    block_list: List[int] = []
    """List of thread ids which should be blocked from execution."""

    def __add__(self, other: Plan) -> Plan:
        """
        Language expression for sequential execution.

        :param other: Another Language expression, either a designator_description or language expression
        :return: A :func:`~Sequential` object which is the new root node of the language tree
        """
        return sequential((self, other))


    def __sub__(self, other: LanguageMixin) -> Plan:
        """
        Language expression for try in order.

        :param other: Another Language expression, either a designator_description or language expression
        :return: A :func:`~TryInOrder` object which is the new root node of the language tree
        """
        return try_in_order((self, other))

    def __or__(self, other: LanguageMixin) -> Plan:
        """
        Language expression for parallel execution.

        :param other: Another Language expression, either a designator_description or language expression
        :return: A :func:`~Parallel` object which is the new root node of the language tree
        """
        return parallel((self, other))


    def __xor__(self, other: LanguageMixin) -> Plan:
        """
        Language expression for try all execution.

        :param other: Another Language expression, either a designator_description or language expression
        :return: A :func:`~TryAll` object which is the new root node of the language tree
        """
        return try_all((self, other))


    def __rshift__(self, other: LanguageMixin):
        """
        Operator for Monitors, this always makes the Monitor the parent of the other expression.
        
        :param other: Another Language expression
        :return: The Monitor which is now the new root node.
        """
        if isinstance(self, MonitorNode) and isinstance(other, MonitorNode):
            raise AttributeError("You can't attach a Monitor to another Monitor.")
        if isinstance(self, MonitorNode):
            self.children = [other]
            return self
        elif isinstance(other, MonitorNode):
            other.children = [self]
            return other

    def __mul__(self, other: int):
        """
        Language expression for Repeated execution. The other attribute of this operator has to be an integer.

        :param other: An integer which states how often the Language expression should be repeated
        :return: A :func:`~Repeat` object which is the new root node of the language tree
        """
        return repeat((self, other), repeat=other)


    def __rmul__(self, other: int):
        """
        Language expression for Repeated execution. The other attribute of this operator has to be an integer. This is
        the reversed operator of __mul__ which allows to write:

        .. code-block:: python
        
            2 * ParkAction()

        :param other: An integer which states how often the Language expression should be repeated
        :return: A :func:`~Repeat` object which is the new root node of the language tree
        """
        return repeat((self, other), repeat=other)

    def simplify(self) -> LanguageMixin:
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
                if isinstance(child, MonitorNode):
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


class LanguagePlanMixins:
    def __enter__(self: Plan):
        self.prev_plan = Plan.current_plan
        Plan.current_plan = self

    def __exit__(self: Plan, exc_type, exc_val, exc_tb):
        Plan.current_plan = self.prev_plan
        if Plan.current_plan:
            Plan.current_plan.mount(self, Plan.current_plan.current_node)


class LanguagePlan(Plan):

    def __init__(self, root: LanguageNode, *children: Plan):
        super().__init__(root=root)
        for child in children:
            self.mount(child)

    def simplify_language_nodes(self):
        for source, target in self.edges:
            if isinstance(source, LanguageNode) and isinstance(target, LanguageNode):
                self.merge_nodes(source, target)

class SequentialPlan(LanguagePlan, LanguagePlanMixins):

    def __init__(self, *children: Plan) -> None:
        seq = SequentialNode()
        super().__init__(root=seq, *children)


class ParallelPlan(LanguagePlan, LanguagePlanMixins):

    def __init__(self, *children: Plan) -> None:
        par = ParallelNode()
        super().__init__(root=par, *children)

class TryInOrderPlan(LanguagePlan, LanguagePlanMixins):

    def __init__(self,  *children: Plan) -> None:
        try_in_order =TryInOrderNode()
        super().__init__(root=try_in_order, *children)


class TryAllPLan(LanguagePlan, LanguagePlanMixins):

    def __init__(self,  *children: Plan) -> None:
        try_all = TryAllNode()
        super().__init__(root=try_all, *children)

class RepeatPlan(LanguagePlan, LanguagePlanMixins):

    def __init__(self, repeat=1,  *children: Plan):
        repeat = RepeatNode(repeat=repeat)
        super().__init__(root=repeat, *children)

class MonitorPlan(LanguagePlan, LanguagePlanMixins):

    def __init__(self, condition,  *children: Plan) -> None:
        monitor = MonitorNode(condition=condition)
        super().__init__(root=monitor, *children)


@dataclass
class LanguageNode(PlanNode):
    ...



@dataclass
class RepeatNode(LanguageNode):
    repeat: int = 1

    """
    Executes all children a given number of times.
    """
    def perform(self):
        """
        Behaviour of repeat, executes all children in a loop as often as stated on initialization.

        :return:
        """
        results = []
        for i in range(self.repeat):
            for child in self.children:
                if self.interrupted:
                    return State.FAILED, results
                try:
                    results.append(child.resolve().perform())
                except PlanFailure as e:
                    self.root.exceptions[self] = e
        return State.SUCCEEDED, results

    def interrupt(self) -> None:
        """
        Stops the execution of this language expression by setting the ``interrupted`` variable to True, adding this
        thread to the block_list in ProcessModule and interrupting the current giskard goal
        """
        self.interrupted = True
        self.block_list.append(threading.get_ident())
        if giskard.giskard_wrapper:
            giskard.giskard_wrapper.interrupt()

    def __hash__(self):
        return id(self)

@dataclass
class MonitorNode(LanguageNode):
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
        self.exception_queue = Queue()
        if callable(condition):
            self.condition = Fluent(condition)
        elif isinstance(condition, Fluent):
            self.condition = condition
        else:
            raise AttributeError("The condition of a Monitor has to be a Callable or a Fluent")

    def perform(self) -> Tuple[State, Any]:
        """
        Behavior of the Monitor, starts a new Thread which checks the condition and then performs the attached language
        expression

        :return: The state of the attached language expression, as well as a list of the results of the children
        """
        def check_condition():
            while not self.kill_event.is_set():
                try:
                    cond = self.condition.get_value()
                    if cond:
                        for child in self.children:
                            try:
                                child.interrupt()
                            except NotImplementedError:
                                pass
                        if isinstance(cond, type) and issubclass(cond, Exception):
                            self.exception_queue.put(cond)
                        else:
                            self.exception_queue.put(PlanFailure("Condition met in Monitor"))
                        return
                except Exception as e:
                    self.exception_queue.put(e)
                    return
                sleep(0.1)

        t = threading.Thread(target=check_condition)
        t.start()
        try:
            state, result = self.children[0].perform()
            if not self.exception_queue.empty():
                raise self.exception_queue.get()
        finally:
            self.kill_event.set()
            t.join()
        return state, result

    def interrupt(self) -> None:
        """
        Calls interrupt for each child
        """
        for child in self.children:
            child.interrupt()

    def __hash__(self):
        return id(self)

@dataclass
class SequentialNode(LanguageNode):
    """
    Executes all children sequentially, an exception while executing a child does not terminate the whole process.
    Instead, the exception is saved to a list of all exceptions thrown during execution and returned.

    Behaviour:
        Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from each
        child's perform() method. The state is :py:attr:`~State.SUCCEEDED` *iff* all children are executed without
        exception. In any other case the State :py:attr:`~State.FAILED` will be returned.
    """

    def perform(self) -> Tuple[State, List[Any]]:
        """
        Behaviour of Sequential, calls perform() on each child sequentially

        :return: The state and list of results according to the behaviour described in :func:`Sequential`
        """
        children_return_values = [None] * len(self.children)
        try:
            for index, child in enumerate(self.children):
                if self.interrupted:
                    if threading.get_ident() in self.block_list:
                        self.block_list.remove(threading.get_ident())
                    return State.FAILED, children_return_values
                self.root.executing_thread[id(child)] = threading.get_ident()
                ret_val = child.resolve().perform()
                if isinstance(ret_val, tuple):
                    child_state, child_result = ret_val
                    children_return_values[index] = child_result
                else:
                    children_return_values[index] = ret_val
        except PlanFailure as e:
            self.root.exceptions[self] = e
            return State.FAILED, children_return_values
        return State.SUCCEEDED, children_return_values

    def interrupt(self) -> None:
        """
        Interrupts the execution of this language expression by setting the ``interrupted`` variable to True and calling
        interrupt on the current giskard goal.
        """
        self.interrupted = True
        self.block_list.append(threading.get_ident())
        if giskard.giskard_wrapper:
            giskard.giskard_wrapper.interrupt()

    def __hash__(self):
        return id(self)

@dataclass
class TryInOrderNode(LanguageNode):
    """
    Executes all children sequentially, an exception while executing a child does not terminate the whole process.
    Instead, the exception is saved to a list of all exceptions thrown during execution and returned.

    Behaviour:
        Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from each
        child's perform() method. The state is :py:attr:`~State.SUCCEEDED` if one or more children are executed without
        exception. In the case that all children could not be executed the State :py:attr:`~State.FAILED` will be returned.
    """

    def perform(self) -> Tuple[State, List[Any]]:
        """
        Behaviour of TryInOrder, calls perform() on each child sequentially and catches raised exceptions.

        :return: The state and list of results according to the behaviour described in :func:`TryInOrder`
        """
        failure_list = []
        children_return_values = [None] * len(self.children)
        for index, child in enumerate(self.children):
            if self.interrupted:
                if threading.get_ident() in self.block_list:
                    self.block_list.remove(threading.get_ident())
                return State.INTERRUPTED, children_return_values
            try:
                ret_val = child.resolve().perform()
                if isinstance(ret_val, tuple):
                    child_state, child_result = ret_val
                    children_return_values[index] = child_result
                else:
                    children_return_values[index] = ret_val
            except PlanFailure as e:
                failure_list.append(e)
        if len(failure_list) > 0:
            self.root.exceptions[self] = failure_list
        if len(failure_list) == len(self.children):
            self.root.exceptions[self] = failure_list
            return State.FAILED, children_return_values
        else:
            return State.SUCCEEDED, children_return_values

    def interrupt(self) -> None:
        """
        Interrupts the execution of this language expression by setting the ``interrupted`` variable to True, adding
        the current thread to the block_list in Language and interrupting the current giskard goal.
        """
        self.interrupted = True
        self.block_list.append(threading.get_ident())
        if giskard.giskard_wrapper:
            giskard.giskard_wrapper.interrupt()

    def __hash__(self):
        return id(self)

@dataclass
class ParallelNode(LanguageNode):
    """
    Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
    exceptions during execution will be caught, saved to a list and returned upon end.

    Behaviour:
        Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from
        each child's perform() method. The state is :py:attr:`~State.SUCCEEDED` *iff* all children could be executed without
        an exception. In any other case the State :py:attr:`~State.FAILED` will be returned.

    """

    def perform(self) -> Tuple[State, List[Any]]:
        """
        Behaviour of Parallel, creates a new thread for each child and calls perform() of the child in the respective
        thread.

        :return: The state and list of results according to the behaviour described in :func:`Parallel`

        """
        results = [None] * len(self.children)
        self.threads: List[threading.Thread] = []
        state = State.SUCCEEDED
        results_lock = threading.Lock()

        def lang_call(child_node, index):
            nonlocal state
            if ("DesignatorDescription" in [cls.__name__ for cls in child_node.__class__.__mro__]
                    and self.__class__.__name__ not in self.do_not_use_giskard):
                if self not in giskard.par_threads.keys():
                    giskard.par_threads[self] = [threading.get_ident()]
                else:
                    giskard.par_threads[self].append(threading.get_ident())
            try:
                self.root.executing_thread[id(child)] = threading.get_ident()
                result = child_node.resolve().perform()
                if isinstance(result, tuple):
                    child_state, child_result = result
                    with results_lock:
                        results[index] = child_result
                else:
                    with results_lock:
                        results[index] = result
            except PlanFailure as e:
                nonlocal state
                with results_lock:
                    state = State.FAILED
                if self in self.root.exceptions.keys():
                    self.root.exceptions[self].append(e)
                else:
                    self.root.exceptions[self] = [e]

        for index, child in enumerate(self.children):
            if self.interrupted:
                state = State.FAILED
                break
            t = threading.Thread(target=lambda: lang_call(child, index))
            t.start()
            self.threads.append(t)
        for thread in self.threads:
            thread.join()
        with results_lock:
            for thread in self.threads:
                if thread.ident in self.block_list:
                    self.block_list.remove(thread.ident)
        if self in self.root.exceptions.keys() and len(self.root.exceptions[self]) != 0:
            state = State.FAILED
        return state, results

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

    def __hash__(self):
        return id(self)

@dataclass
class TryAllNode(LanguageNode):
    """
    Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
    exceptions during execution will be caught, saved to a list and returned upon end.

    Behaviour:
        Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from each
        child's perform() method. The state is :py:attr:`~State.SUCCEEDED` if one or more children could be executed
        without raising an exception. If all children fail the State :py:attr:`~State.FAILED` will be returned.
    """

    def perform(self) -> Tuple[State, List[Any]]:
        """
        Behaviour of TryAll, creates a new thread for each child and executes all children in their respective threads.

        :return: The state and list of results according to the behaviour described in :func:`TryAll`
        """
        results = [None] * len(self.children)
        results_lock = threading.Lock()
        state = State.SUCCEEDED
        self.threads: List[threading.Thread] = []
        failure_list = []

        def lang_call(child_node, index):
            if ("DesignatorDescription" in [cls.__name__ for cls in child_node.__class__.__mro__]
                    and self.__class__.__name__ not in self.do_not_use_giskard):
                if self not in giskard.par_threads.keys():
                    giskard.par_threads[self] = [threading.get_ident()]
                else:
                    giskard.par_threads[self].append(threading.get_ident())
            try:
                result = child_node.resolve().perform()
                if isinstance(result, tuple):
                    child_state, child_result = result
                    with results_lock:
                        results[index] = child_result
                else:
                    with results_lock:
                        results[index] = result
            except PlanFailure as e:
                failure_list.append(e)
                if self in self.root.exceptions.keys():
                    self.root.exceptions[self].append(e)
                else:
                    self.root.exceptions[self] = [e]
        for index, child in enumerate(self.children):
            if self.interrupted:
                state = State.FAILED
                break
            t = threading.Thread(target=lambda: lang_call(child, index))
            self.threads.append(t)
            t.start()
        for thread in self.threads:
            thread.join()
        with results_lock:
            for thread in self.threads:
                if thread.ident in self.block_list:
                    self.block_list.remove(thread.ident)
        if len(self.children) == len(failure_list):
            self.root.exceptions[self] = failure_list
            state = State.FAILED
        return state, results

    def interrupt(self) -> None:
        """
        Interrupts the execution of this language expression by setting the ``interrupted`` variable to True, adding the
        thread id of all parallel execution threads to the block_list in Language and interrupting the current giskard
        """
        self.interrupted = True
        self.block_list += [t.ident for t in self.threads]
        if giskard.giskard_wrapper:
            giskard.giskard_wrapper.interrupt()

    def __hash__(self):
        return id(self)

@dataclass
class CodeNode(LanguageNode):
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
        self.performable = self.__class__

    def execute(self) -> Any:
        """
        Execute the code with its arguments

        :returns: State.SUCCEEDED, and anything that the function associated with this object will return.
        """
        child_state = State.SUCCEEDED
        ret_val = self.function(**self.kwargs)
        if isinstance(ret_val, tuple):
            child_state, child_result = ret_val
        else:
            child_result = ret_val

        return child_state, child_result

    def resolve(self) -> Self:
        return self

    def interrupt(self) -> None:
        raise NotImplementedError

    def __hash__(self):
        return id(self)


