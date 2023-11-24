# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from typing import Type, Iterable

import rospy
from anytree import NodeMixin, Node, PreOrderIter

from .enums import State
import threading

from .plan_failures import PlanFailure, NotALanguageExpression
from .external_interfaces import giskard

class Language(NodeMixin):
    """
    Parent class for language expressions. Implements the operators as well as methods to reduce the resulting language
    tree.
    """

    def __init__(self, parent: NodeMixin = None, children: Iterable[NodeMixin] = None):
        """
        Default constructor for anytree nodes. If the parent is none this is the root node.

        :param parent: The parent node of this node
        :param children: All children of this node as a tuple oder iterable
        """
        self.parent = parent
        self.exceptions = []
        if children:
            self.children = children

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
            raise NotALanguageExpression(f"Only classes that inherit from the Language class can be used with the plan language, these are usually Designators or Code objects. \nThe object '{other}' does not inherit from the Language class.")
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
        return Parallel(parent=None, children=(self, other)).simplify()

    # def __truediv__(self, other: Language) -> Pursue:
    #     """
    #     Language expression for purse execution.
    #
    #     :param other: Another Language expression, either a designator or language expression
    #     :return: A :func:`~Pursue` object which is the new root node of the language tree
    #     """
    #     if not issubclass(other.__class__, Language):
    #         raise NotALanguageExpression(
    #             f"Only classes that inherit from the Language class can be used with the plan language, these are usually Designators or Code objects. \nThe object '{other}' does not inherit from the Language class.")
    #     return Pursue(parent=None, children=(self, other)).simplify()

    def __xor__(self, other: Language) -> TryAll:
        """
        Language expression for try all execution.

        :param other: Another Language expression, either a designator or language expression
        :return: A :func:`~TryAll` object which is the new root node of the language tree
        """
        if not issubclass(other.__class__, Language):
            raise NotALanguageExpression(
                f"Only classes that inherit from the Language class can be used with the plan language, these are usually Designators or Code objects. \nThe object '{other}' does not inherit from the Language class.")
        return TryAll(parent=None, children=(self, other)).simplify()

    def simplify(self) -> Language:
        """
        Simplifies the language tree by merging which have a parent-child relation and are of the same type.

        .. code-block:: python

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
                child.resolve().perform()
        except PlanFailure as e:
            self.root.exceptions.append(e)
            return State.FAILED
        return State.SUCCEEDED


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
            try:
                child.resolve().perform()
            except PlanFailure as e:
                failure_list.append(e)
        if len(failure_list) == len(self.children):
            self.root.exceptions.append(failure_list)
            return State.FAILED
        else:
            return State.SUCCEEDED


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
        threads = []

        def lang_call(child_node):
            # if isinstance(child_node, DesignatorDescription):
            if "DesignatorDescription" in [cls.__name__ for cls in child_node.__class__.__mro__]:
                if self not in giskard.par_threads.keys():
                    giskard.par_threads[self] = [threading.get_ident()]
                else:
                    giskard.par_threads[self].append(threading.get_ident())
            child_node.resolve().perform()

        for child in self.children:
            t = threading.Thread(target=lambda: lang_call(child))
            try:
                t.start()
            except PlanFailure as e:
                for thread in threads:
                    thread.join()
                self.root.exceptions.append(e)
                return State.FAILED
            threads.append(t)
        for thread in threads:
            thread.join()
        return State.SUCCEEDED


class Pursue(Language):
    """
    Executes all children in parallel by creating a thread per children and executing them in the respective thread. As
    soon as one child finishes successfully the execution of all other children will be interrupted.
    All exceptions during execution will be caught, saved to a list and returned upon end.

    Behaviour:
        Returns the State :py:attr:`~State.SUCCEEDED` if the first child to finish did not raise an exception and
        returns the State :py:attr:`~State.FAILED` if the first child to finish raise an exception.
    """

    def perform(self) -> State:
        """
        Behaviour of Pursue, creates a new thread for each child and calls perform() of the child in the respective
        thread. Interrupts all threads if the one thread finishes.

        :return: The state according to the behaviour described in :func:`Pursue`
        """
        ...


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
        threads = []
        failure_list = []
        giskard.number_of_par_goals = len(self.children)
        for child in self.children:
            t = threading.Thread(target=child.resolve().perform)
            try:
                t.start()
            except PlanFailure as e:
                failure_list.append(e)
            threads.append(t)
        for thread in threads:
            thread.join()
        if len(self.children) == len(failure_list):
            self.root.exceptions.append(failure_list)
            return State.FAILED
        else:
            return State.SUCCEEDED
