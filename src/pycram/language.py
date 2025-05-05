# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from dataclasses import dataclass, field
from queue import Queue
from typing_extensions import Iterable, Optional, Callable, Dict, Any, List, Union, Tuple, Self, Sequence

from .datastructures.enums import TaskStatus
import threading

from .fluent import Fluent
from .failures import PlanFailure
from .external_interfaces import giskard
from .ros import sleep, loginfo
from .plan import PlanNode, Plan, managed_node


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
    """
    Base class for language plans
    """

    def __init__(self, root: LanguageNode, *children: Plan):
        """
        Creates a Language plan with the given root node and children. The root node als determines the behavior of the
        language plan

        :param root: A LanguageNode which should be the root
        :param children: A list of child nodes which should be added to the plan
        """
        super().__init__(root=root)
        for child in children:
            self.mount(child)
        self.simplify_language_nodes()

    def simplify_language_nodes(self):
        """
        Traverses the plan and merges LanguageNodes of the same type
        """
        to_be_merged = []
        for source, target in self.edges:
            if isinstance(source, LanguageNode) and isinstance(target, LanguageNode) and type(source) == type(target):
                to_be_merged.append((source, target))
        # Since merging nodes changes the edges of the plan we do this in two steps
        for source, target in to_be_merged:
            self.merge_nodes(source, target)

class SequentialPlan(LanguagePlan):
    """
    Creates a plan which executes its children in sequential order
    """

    def __init__(self, *children: Plan) -> None:
        seq = SequentialNode()
        super().__init__(seq, *children)


class ParallelPlan(LanguagePlan):
    """
    Creates a plan which executes all children in parallel in seperate threads
    """
    parallel_blocklist = ["PickUpAction", "PlaceAction", "OpenAction", "CloseAction", "TransportAction",
                          "GraspingAction"]
    """
    A list of Actions which can't be part of a Parallel plan
    """
    def __init__(self, *children: Plan, root: LanguageNode = None) -> None:
        root = root or ParallelNode()
        for child in children:
            child_actions = [action.designator_ref.performable.__name__ for action in child.actions]
            for action in child_actions:
                if action in self.parallel_blocklist:
                    raise AttributeError(f"You can't create a ParallelPlan with a {child.__class__.__name__}.")

        super().__init__(root, *children)

class TryInOrderPlan(LanguagePlan):
    """
    Creates a plan that executes all children in sequential order but does not stop if one of them throws an error
    """

    def __init__(self,  *children: Plan) -> None:
        try_in_order = TryInOrderNode()
        super().__init__(try_in_order, *children)

class TryAllPLan(ParallelPlan):
    """
    Creates a plan which executes all children in parallel but does not abort if one throws an error
    """

    def __init__(self,  *children: Plan) -> None:
        try_all = TryAllNode()
        super().__init__(root=try_all, *children)

class RepeatPlan(LanguagePlan):
    """
    A plan which repeats all children a number of times
    """

    def __init__(self, repeat=1,  *children: Plan):
        if not isinstance(repeat, int):
            raise AttributeError(f"Repeat must be an integer")
        repeat = RepeatNode(repeat=repeat)
        super().__init__(repeat, *children)

class MonitorPlan(LanguagePlan):
    """
    A plan which monitors a condition and upon the condition becoming true interrupts all children
    """

    def __init__(self, condition,  *children: Plan) -> None:
        monitor = MonitorNode(condition=condition)
        super().__init__(monitor, *children)

class CodePlan(Plan):
    """
    A Plan that contains a function to be executed. Mainly intended for debugging purposes
    """
    def __init__(self, func: Callable, kwargs: Dict[str, Any] = None) -> None:
        kwargs = kwargs or {}
        code = CodeNode(func, kwargs)
        super().__init__(code)


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
        child's perform() method. The state is :py:attr:`~TaskStatus.SUCCEEDED` *iff* all children are executed without
        exception. In any other case the State :py:attr:`~TaskStatus.FAILED` will be returned.
    """

    def perform(self):
        """
        Behaviour of Sequential, calls perform() on each child sequentially

        :return: The state and list of results according to the behaviour described in :func:`Sequential`
        """
        try:
            loginfo(f"Executing {self}")
            self.perform_sequential(self.children)
            self.status = TaskStatus.SUCCEEDED
        except PlanFailure as e:
            self.status = TaskStatus.FAILED
            self.reason = e
            # Failure Handling could be done here
            raise e


    def perform_sequential(self, nodes: List[PlanNode], raise_exceptions = True) -> Any:
        """
        Behavior of the sequential node, performs all children in sequence and raises error if they occur.

        :param nodes: A list of nodes which should be performed in sequence
        :param raise_exceptions: If True (default) errors will be raised
        """
        res = None
        for child in nodes:
            try:
                res = child.perform()
            except PlanFailure as e:
                self.status = TaskStatus.FAILED
                self.reason = e
                if raise_exceptions:
                    raise e

        return res

    def __hash__(self):
        return id(self)

    def __repr__(self):
        return f"{self.__class__.__name__}"


@dataclass
class ParallelNode(LanguageNode):
    """
    Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
    exceptions during execution will be caught, saved to a list and returned upon end.

    Behaviour:
        Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from
        each child's perform() method. The state is :py:attr:`~TaskStatus.SUCCEEDED` *iff* all children could be executed without
        an exception. In any other case the State :py:attr:`~TaskStatus.FAILED` will be returned.

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
        """
        Behaviour of the parallel node performs the given nodes in parallel in different threads.

        :param nodes: A list of nodes which should be performed in parallel
        """
        threads = []
        for child in nodes:
            t = threading.Thread(target=self._lang_call, args=[child])
            t.start()
            threads.append(t)
        for thread in threads:
            thread.join()

    def _lang_call(self, node: PlanNode):
        """
        Wrapper method which is executed in the thread. Wraps the given node in a try catch and performs it

        :param node: The node which is to be performed
        """
        try:
            return node.perform()
        except PlanFailure as e:
            self.status = TaskStatus.FAILED
            self.reason = e
            # Failure handling comes here
            raise e

    def __hash__(self):
        return id(self)

    def __repr__(self):
        return f"{self.__class__.__name__}"



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
        child's perform() method. The state is :py:attr:`~TaskStatus.SUCCEEDED` if one or more children are executed without
        exception. In the case that all children could not be executed the State :py:attr:`~TaskStatus.FAILED` will be returned.
    """

    def perform(self):
        """
        Behaviour of TryInOrder, calls perform() on each child sequentially and catches raised exceptions.

        :return: The state and list of results according to the behaviour described in :func:`TryInOrder`
        """
        self.perform_sequential(self.children, raise_exceptions=False)
        child_statuses = [child.status for child in self.children]
        self.status = TaskStatus.SUCCEEDED if TaskStatus.SUCCEEDED in child_statuses else TaskStatus.FAILED
        child_results = list(filter(None, [child.result for child in self.recursive_children]))
        if child_results:
            self.result = child_results[0]
            return self.result[0]

    def __hash__(self):
        return id(self)


@dataclass
class TryAllNode(ParallelNode):
    """
    Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
    exceptions during execution will be caught, saved to a list and returned upon end.

    Behaviour:
        Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from each
        child's perform() method. The state is :py:attr:`~TaskStatus.SUCCEEDED` if one or more children could be executed
        without raising an exception. If all children fail the State :py:attr:`~TaskStatus.FAILED` will be returned.
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

    @managed_node
    def execute(self) -> Any:
        """
        Execute the code with its arguments

        :returns: Anything that the function associated with this object will return.
        """
        ret_val = self.function(**self.kwargs)
        if isinstance(ret_val, tuple):
            child_state, child_result = ret_val
        else:
            child_result = ret_val

        return child_result

    def resolve(self) -> Self:
        return self

    def __hash__(self):
        return id(self)


