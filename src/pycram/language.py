# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import atexit
import inspect
import logging
import threading
from dataclasses import dataclass, field
from queue import Queue

from typing_extensions import (
    Optional,
    Callable,
    Dict,
    Any,
    List,
    Union,
    Self,
    Type,
    TYPE_CHECKING,
)

from .datastructures.dataclasses import Context
from .datastructures.enums import TaskStatus, MonitorBehavior
from .datastructures.partial_designator import PartialDesignator
from .failures import PlanFailure
from .fluent import Fluent
from .plan import (
    PlanNode,
    Plan,
    managed_node,
    ActionNode,
    MotionNode,
    ResolvedActionNode,
)
from .ros import sleep

if TYPE_CHECKING:
    from .robot_plans.actions.base import ActionDescription

    from .robot_plans import BaseMotion

logger = logging.getLogger(__name__)


class LanguagePlan(Plan):
    """
    Base class for language plans
    """

    def __init__(
        self,
        root: LanguageNode,
        context: Context,
        *children: Union[Plan, PartialDesignator, BaseMotion, ActionDescription],
    ):
        """
        Creates a Language plan with the given root node and children. The root node als determines the behavior of the
        language plan

        :param root: A LanguageNode which should be the root
        :param children: A list of child nodes which should be added to the plan
        :param world: The world in which the plan is executed
        :param plan: A plan of which this plan is a part of
        :param robot_view: The robot which is executing the plan
        """
        super().__init__(root=root, context=context)
        for child in children:
            if isinstance(child, Plan):
                self.mount(child, self.root)
            elif isinstance(child, PartialDesignator):
                node = ActionNode(
                    designator_ref=child,
                    designator_type=child.performable,
                    kwargs=child.kwargs,
                )
                self.add_edge(self.root, node)
            elif "ActionDescription" in [c.__name__ for c in child.__class__.__mro__]:
                node = ResolvedActionNode(
                    designator_ref=child,
                    designator_type=child.__class__,
                    kwargs=child.__dict__,
                )
                self.add_edge(self.root, node)
            elif "BaseMotion" in [c.__name__ for c in child.__class__.__mro__]:
                kwargs = (
                    inspect.signature(child.__init__).bind(**child.__dict__).arguments
                )
                node = MotionNode(
                    designator_ref=child, designator_type=child.__class__, kwargs=kwargs
                )
                self.add_edge(self.root, node)
            else:
                ValueError(
                    f"Type: {child.__class__.__name__} is not supported as child of a LanguagePlan"
                )
        self.simplify_language_nodes()

    def simplify_language_nodes(self):
        """
        Traverses the plan and merges LanguageNodes of the same type
        """
        to_be_merged = []
        for source, target in self.edges:
            if (
                isinstance(source, LanguageNode)
                and isinstance(target, LanguageNode)
                and type(source) == type(target)
            ):
                to_be_merged.append((source, target))
        # Since merging nodes changes the edges of the plan we do this in two steps
        for source, target in to_be_merged:
            self.merge_nodes(source, target)


class SequentialPlan(LanguagePlan):
    """
    Creates a plan which executes its children in sequential order
    """

    def __init__(
        self, context: Context, *children: Union[Plan, PartialDesignator, BaseMotion]
    ) -> None:
        seq = SequentialNode()
        super().__init__(seq, context, *children)


class ParallelPlan(LanguagePlan):
    """
    Creates a plan which executes all children in parallel in seperate threads
    """

    parallel_blocklist = [
        "PickUpAction",
        "PlaceAction",
        "OpenAction",
        "CloseAction",
        "TransportAction",
        "GraspingAction",
    ]
    """
    A list of Actions which can't be part of a Parallel plan
    """

    def __init__(
        self,
        context: Context,
        *children: Union[Plan, PartialDesignator, BaseMotion],
        root: LanguageNode = None,
    ) -> None:
        root = root or ParallelNode()
        all_child_actions = []
        for child in children:
            if isinstance(child, Plan):
                all_child_actions.extend(
                    [
                        action.designator_ref.performable.__name__
                        for action in child.actions
                    ]
                )
            elif isinstance(child, PartialDesignator):
                all_child_actions.append(child.performable.__name__)
            elif "BaseMotion" in [c.__name__ for c in child.__class__.__mro__]:
                all_child_actions.append(child.__class__.__name__)

        for action in all_child_actions:
            if action in self.parallel_blocklist:
                raise AttributeError(
                    f"You can't create a ParallelPlan with a {action}."
                )

        super().__init__(root, context, *children)


class TryInOrderPlan(LanguagePlan):
    """
    Creates a plan that executes all children in sequential order but does not stop if one of them throws an error
    """

    def __init__(
        self, context: Context, *children: Union[Plan, PartialDesignator, BaseMotion]
    ) -> None:
        try_in_order = TryInOrderNode()
        super().__init__(try_in_order, context, *children)


class TryAllPLan(ParallelPlan):
    """
    Creates a plan which executes all children in parallel but does not abort if one throws an error
    """

    def __init__(
        self, context: Context, *children: Union[Plan, PartialDesignator, BaseMotion]
    ) -> None:
        try_all = TryAllNode()
        super().__init__(context, *children, root=try_all)


class RepeatPlan(LanguagePlan):
    """
    A plan which repeats all children a number of times
    """

    def __init__(
        self,
        context: Context,
        repeat=1,
        *children: Union[Plan, PartialDesignator, BaseMotion],
    ):
        if not isinstance(repeat, int):
            raise AttributeError(f"Repeat must be an integer")
        repeat = RepeatNode(repeat=repeat)
        super().__init__(repeat, context, *children)


class MonitorPlan(LanguagePlan):
    """
    A plan which monitors a condition and upon the condition becoming true interrupts all children. Monitors can have
    different behaviors, they can Interrupt, Pause or Resume the execution of the children. If the behavior is set to
    resume the plan will be paused until the condition is met.

    :param condition: A condition which should be monitored
    :behavior: The behavior of the monitor, either :py:attr:`~MonitorBehavior.INTERRUPT`, :py:attr:`~MonitorBehavior.PAUSE` or :py:attr:`~MonitorBehavior.RESUME`
    """

    def __init__(
        self,
        condition,
        context: Context,
        *children: Union[Plan, PartialDesignator, BaseMotion],
        behavior=MonitorBehavior.INTERRUPT,
    ) -> None:
        monitor = MonitorNode(condition=condition, behavior=behavior)
        super().__init__(monitor, context, *children)


class CodePlan(LanguagePlan):
    """
    A Plan that contains a function to be executed. Mainly intended for debugging purposes
    """

    def __init__(
        self, context: Context, func: Callable, kwargs: Dict[str, Any] = None
    ) -> None:
        kwargs = kwargs or {}
        code = CodeNode(func, kwargs)
        super().__init__(code, context)


@dataclass
class LanguageNode(PlanNode):
    designator_type: Type[LanguageNode] = field(default_factory=lambda: LanguageNode)
    """
    Superclass for language nodes in a plan. Used to distinguish language nodes from other types of nodes.
    """


@dataclass
class SequentialNode(LanguageNode):
    designator_type: Type[SequentialNode] = field(
        default_factory=lambda: SequentialNode
    )
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
        result = None
        try:
            logger.info(f"Executing {self}")
            result = self.perform_sequential(self.children)
            self.status = TaskStatus.SUCCEEDED
        except PlanFailure as e:
            self.status = TaskStatus.FAILED
            self.reason = e
            # Failure Handling could be done here
            raise e
        return result

    def perform_sequential(self, nodes: List[PlanNode], raise_exceptions=True) -> Any:
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
        self.status = (
            TaskStatus.SUCCEEDED
            if TaskStatus.FAILED not in child_statuses
            else TaskStatus.FAILED
        )

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

    @managed_node
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
        Monitors start a new Thread which checks the condition while performing the nodes below it. Monitors can have
        different behaviors, they can Interrupt, Pause or Resume the execution of the children.
        If the behavior is set to Resume the plan will be paused until the condition is met.
    """

    def __init__(
        self,
        condition: Union[Callable, Fluent] = None,
        behavior: Optional[MonitorBehavior] = MonitorBehavior.INTERRUPT,
    ):
        """
        When initializing a Monitor a condition must be provided. The condition is a callable or a Fluent which returns \
        True or False.

        :param condition: The condition upon which the Monitor should interrupt the attached language expression.
        """
        super().__init__()
        self.kill_event = threading.Event()
        self.exception_queue = Queue()
        self.behavior = behavior
        if self.behavior == MonitorBehavior.RESUME:
            self.pause()
        if callable(condition):
            self.condition = Fluent(condition)
        elif isinstance(condition, Fluent):
            self.condition = condition
        else:
            raise AttributeError(
                "The condition of a Monitor has to be a Callable or a Fluent"
            )
        self.monitor_thread = threading.Thread(
            target=self.monitor, name=f"MonitorThread-{id(self)}"
        )
        self.monitor_thread.start()

    @managed_node
    def perform(self):
        """
        Behavior of the Monitor, starts a new Thread which checks the condition and then performs the attached language
        expression

        :return: The state of the attached language expression, as well as a list of the results of the children
        """
        self.perform_sequential(self.children)
        self.kill_event.set()
        self.monitor_thread.join()

    def monitor(self):
        atexit.register(self.kill_event.set)
        while not self.kill_event.is_set():
            if self.condition.get_value():
                if self.behavior == MonitorBehavior.INTERRUPT:
                    self.interrupt()
                    self.kill_event.set()
                elif self.behavior == MonitorBehavior.PAUSE:
                    self.pause()
                    self.kill_event.set()
                elif self.behavior == MonitorBehavior.RESUME:
                    self.resume()
                    self.kill_event.set()
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

    @managed_node
    def perform(self):
        """
        Behaviour of TryInOrder, calls perform() on each child sequentially and catches raised exceptions.

        :return: The state and list of results according to the behaviour described in :func:`TryInOrder`
        """
        self.perform_sequential(self.children, raise_exceptions=False)
        child_statuses = [child.status for child in self.children]
        self.status = (
            TaskStatus.SUCCEEDED
            if TaskStatus.SUCCEEDED in child_statuses
            else TaskStatus.FAILED
        )
        child_results = list(
            filter(None, [child.result for child in self.recursive_children])
        )
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
        self.status = (
            TaskStatus.SUCCEEDED
            if TaskStatus.SUCCEEDED in child_statuses
            else TaskStatus.FAILED
        )

    def __hash__(self):
        return id(self)


@dataclass
class CodeNode(LanguageNode):
    """
    Executable code block in a plan.

    :ivar function: The function (plan) that was called
    :ivar kwargs: Dictionary holding the keyword arguments of the function
    """

    designator_type: Type[LanguageNode] = field(default_factory=lambda: LanguageNode)

    def __init__(
        self, function: Optional[Callable] = None, kwargs: Optional[Dict] = None
    ):
        """
        Initialize a code call

        :param function: The function that was called
        :param kwargs: The keyword arguments of the function as dict
        """
        super().__init__()
        self.function: Callable = function

        if kwargs is None:
            kwargs = dict()
        self.kwargs: Dict[str, Any] = kwargs
        self.perform = self.execute
        self.performable = self.__class__
        self.action = self.__class__

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
