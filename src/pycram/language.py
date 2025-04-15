# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from dataclasses import dataclass, field
from queue import Queue
from typing_extensions import Iterable, Optional, Callable, Dict, Any, List, Union, Tuple, Self, Sequence

from .datastructures.enums import State, TaskStatus
import threading

from .fluent import Fluent
from .failures import PlanFailure
from .external_interfaces import giskard
from .ros import  sleep
from .plan import PlanNode, Plan


class LanguageMixin:
    """
    Parent class for language expressions. Implements the operators as well as methods to reduce the resulting language
    tree.
    """

    def add_language_plan(self: Plan, child_plans: Iterable[Plan], root_node: LanguageNode) -> Plan:
        lang_plan = Plan(root_node)
        lang_plan.mount(self)
        for plan in child_plans:
            lang_plan.mount(plan)

        plan = Plan.current_plan
        plan.mount(lang_plan, plan.current_node)
        return lang_plan

    def __add__(self, other: Plan) -> Plan:
        """
        Language expression for sequential execution.

        :param other: Another Language expression, either a designator_description or language expression
        :return: A :func:`~Sequential` object which is the new root node of the language tree
        """
        return self.add_language_plan(other, SequentialNode())


    def __sub__(self, other: Plan) -> Plan:
        """
        Language expression for try in order.

        :param other: Another Language expression, either a designator_description or language expression
        :return: A :func:`~TryInOrder` object which is the new root node of the language tree
        """
        return self.add_language_plan(other, TryInOrderNode())

    def __or__(self, other: Plan) -> Plan:
        """
        Language expression for parallel execution.

        :param other: Another Language expression, either a designator_description or language expression
        :return: A :func:`~Parallel` object which is the new root node of the language tree
        """
        return self.add_language_plan(other, ParallelNode())


    def __xor__(self, other: Plan) -> Plan:
        """
        Language expression for try all execution.

        :param other: Another Language expression, either a designator_description or language expression
        :return: A :func:`~TryAll` object which is the new root node of the language tree
        """
        return self.add_language_plan(other, TryAllNode())


    def __rshift__(self, other: Callable):
        """
        Operator for Monitors, this always makes the Monitor the parent of the other expression.
        
        :param other: Another Language expression
        :return: The Monitor which is now the new root node.
        """
        if isinstance(self, MonitorNode) and isinstance(other, MonitorNode):
            raise AttributeError("You can't attach a Monitor to another Monitor.")
        return self.add_language_plan((self), MonitorPlan(other))

    def __mul__(self: Plan, other: int):
        """
        Language expression for Repeated execution. The other attribute of this operator has to be an integer.

        :param other: An integer which states how often the Language expression should be repeated
        :return: A :func:`~Repeat` object which is the new root node of the language tree
        """
        return self.add_language_plan(self, RepeatNode(repeat=other))


    def __rmul__(self: Plan, other: int):
        """
        Language expression for Repeated execution. The other attribute of this operator has to be an integer. This is
        the reversed operator of __mul__ which allows to write:

        .. code-block:: python
        
            2 * ParkAction()

        :param other: An integer which states how often the Language expression should be repeated
        :return: A :func:`~Repeat` object which is the new root node of the language tree
        """
        return self.add_language_plan(self, RepeatNode(repeat=other))

    def interrupt(self) -> None:
        """
        Base method for interrupting the execution of Language expression. To be overwritten in a sub-class.
        """
        raise NotImplementedError


class LanguagePlan(Plan):

    def __init__(self, root: LanguageNode, *children: Plan):
        super().__init__(root=root)
        for child in children:
            self.mount(child)

    def simplify_language_nodes(self):
        for source, target in self.edges:
            if isinstance(source, LanguageNode) and isinstance(target, LanguageNode):
                self.merge_nodes(source, target)

class SequentialPlan(LanguagePlan):

    def __init__(self, *children: Plan) -> None:
        seq = SequentialNode()
        super().__init__(seq, *children)


class ParallelPlan(LanguagePlan):
    parallel_blocklist = ["PickUpAction", "PlaceAction", "OpenAction", "CloseAction", "TransportAction",
                          "GraspingAction"]
    def __init__(self, *children: Plan, root: LanguageNode = None) -> None:
        root = ParallelNode() or root
        for child in children:
            if child.__class__.__name__ in self.parallel_blocklist:
                raise TypeError(f"You can't create a ParallelPlan with a {child.__class__.__name__}.")

        super().__init__(root, *children)

class TryInOrderPlan(LanguagePlan):

    def __init__(self,  *children: Plan) -> None:
        try_in_order =TryInOrderNode()
        super().__init__(try_in_order, *children)

class TryAllPLan(ParallelPlan):

    def __init__(self,  *children: Plan) -> None:
        try_all = TryAllNode()
        super().__init__(root=try_all, *children)

class RepeatPlan(LanguagePlan):

    def __init__(self, repeat=1,  *children: Plan):
        repeat = RepeatNode(repeat=repeat)
        super().__init__(repeat, *children)

class MonitorPlan(LanguagePlan):

    def __init__(self, condition,  *children: Plan) -> None:
        monitor = MonitorNode(condition=condition)
        super().__init__(monitor, *children)


@dataclass
class LanguageNode(PlanNode):
    """
    Superclass for language nodes in a plan. Used to distinguish language nodes from other types of nodes.
    """
    ...


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

    def perform(self):
        """
        Behaviour of Sequential, calls perform() on each child sequentially

        :return: The state and list of results according to the behaviour described in :func:`Sequential`
        """
        try:
            self.perform_sequential(self.children)
        except PlanFailure as e:
            self.status = TaskStatus.FAILED
            self.reason = e
            raise e
        self.status = TaskStatus.SUCCEEDED

    @staticmethod
    def perform_sequential(nodes: List[PlanNode], raise_exceptions = True) -> Any:
        results = {}
        for child in nodes:
            try:
                results[child]  = child.perform()
                child.status = TaskStatus.SUCCEEDED
            except PlanFailure as e:
                child.status = TaskStatus.FAILED
                child.reason = e
                if raise_exceptions:
                    raise e

    def __hash__(self):
        return id(self)


@dataclass
class ParallelNode(LanguageNode):
    results: dict = field(default_factory=dict)
    """
    Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
    exceptions during execution will be caught, saved to a list and returned upon end.

    Behaviour:
        Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from
        each child's perform() method. The state is :py:attr:`~State.SUCCEEDED` *iff* all children could be executed without
        an exception. In any other case the State :py:attr:`~State.FAILED` will be returned.

    """

    def perform(self):
        """
        Behaviour of Parallel, creates a new thread for each child and calls perform() of the child in the respective
        thread.

        :return: The state and list of results according to the behaviour described in :func:`Parallel`

        """
        self.perform_parallel(self.children)
        child_statuses = [child.status for child in self.children]
        self.status = TaskStatus.SUCCEEDED if TaskStatus.FAILED not in child_statuses else TaskStatus.FAILED

    def perform_parallel(self, nodes: List[PlanNode]):
        threads = []
        for child in nodes:
            t = threading.Thread(target=self._lang_call, args=[child])
            t.start()
            threads.append(t)
        for thread in threads:
            thread.join()

    def _lang_call(self, node: PlanNode):
        try:
            self.results[node] = node.perform()
            node.state = State.SUCCEEDED
        except PlanFailure as e:
            node.status = TaskStatus.FAILED
            node.reason = e
            raise e


@dataclass
class RepeatNode(SequentialNode):
    repeat: int = 1

    """
    Executes all children a given number of times in sequential order.
    """
    def perform(self):
        """
        Behaviour of repeat, executes all children in a loop as often as stated on initialization.

        :return:
        """
        for _ in range(self.repeat):
            try:
                self.perform_sequential(self.children)
            except PlanFailure as e:
                self.status = TaskStatus.FAILED
                self.reason = e
                raise e

    def __hash__(self):
        return id(self)

@dataclass
class MonitorNode(SequentialNode):
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
        self.kill_event = threading.Event()
        self.exception_queue = Queue()
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

        :return: The state of the attached language expression, as well as a list of the results of the children
        """
        monitor_thread = threading.Thread(target=self.monitor)
        monitor_thread.start()
        self.perform_sequential(self.children)
        self.kill_event.set()
        monitor_thread.join()

    def monitor(self):
        while not self.kill_event.is_set():
            if self.condition.get_value():
                self.interrupt()
            sleep(0.1)

    def __hash__(self):
        return id(self)

@dataclass
class TryInOrderNode(SequentialNode):
    """
    Executes all children sequentially, an exception while executing a child does not terminate the whole process.
    Instead, the exception is saved to a list of all exceptions thrown during execution and returned.

    Behaviour:
        Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from each
        child's perform() method. The state is :py:attr:`~State.SUCCEEDED` if one or more children are executed without
        exception. In the case that all children could not be executed the State :py:attr:`~State.FAILED` will be returned.
    """

    def perform(self):
        """
        Behaviour of TryInOrder, calls perform() on each child sequentially and catches raised exceptions.

        :return: The state and list of results according to the behaviour described in :func:`TryInOrder`
        """
        self.perform_sequential(self.children, raise_exceptions=False)
        child_statuses = [child.status for child in self.children]
        self.status = TaskStatus.SUCCEEDED if TaskStatus.SUCCEEDED in child_statuses else TaskStatus.FAILED

    def __hash__(self):
        return id(self)


@dataclass
class TryAllNode(ParallelNode):
    """
    Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
    exceptions during execution will be caught, saved to a list and returned upon end.

    Behaviour:
        Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from each
        child's perform() method. The state is :py:attr:`~State.SUCCEEDED` if one or more children could be executed
        without raising an exception. If all children fail the State :py:attr:`~State.FAILED` will be returned.
    """

    def perform(self):
        """
        Behaviour of TryAll, creates a new thread for each child and executes all children in their respective threads.

        :return: The state and list of results according to the behaviour described in :func:`TryAll`
        """
        self.perform_parallel(self.children)
        child_statuses = [child.status for child in self.children]
        self.status = TaskStatus.SUCCEEDED if TaskStatus.SUCCEEDED in child_statuses else TaskStatus.FAILED

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

    def __hash__(self):
        return id(self)


