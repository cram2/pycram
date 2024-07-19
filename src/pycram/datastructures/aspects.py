# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from abc import abstractmethod

from .enums import Arms
from .pose import Pose
from typing_extensions import List, Iterable, Dict, Any, Callable
from anytree import NodeMixin, PreOrderIter, Node

from ..designator import ObjectDesignatorDescription, ActionDesignatorDescription
from ..world_concepts.world_object import Object


def managed_io(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        print("test")
        return func(*args, **kwargs)

    return wrapper


class Aspect(NodeMixin):
    """
    Parent class to represent a semantic aspect as part of a knowledge pre-condition of designators.
    Aspects can be combined using logical operators to create complex pre-conditions, the resulting expression is a
    datastructure of a tree.
    """
    resolved_aspect_function: Callable
    """
    Reference to the actual implementation of the aspect function in the KnowledgeSource. This reference is used when 
    evaluating the tree structure of aspects.
    """
    variables: Dict[str, Any]
    """
    Dictionary of variables and their values which are used in the aspect tree. This dictionary is only to be used in 
    the root node.
    """

    def __init__(self, parent: NodeMixin = None, children: Iterable[NodeMixin] = None,
                 input: str = None, output: str = None):
        super().__init__()
        self.parent = parent
        self.input = input
        self.output = output
        self.variables = {}
        if children:
            self.children = children

    def __and__(self, other):
        """
        Overload the and operator to create an AndAspect as part of the tree structure.

        :param other: The other aspect to combine with
        :return: An AndAspect containing both, this and the other, aspect
        """
        return AndAspect([self, other])

    def __or__(self, other):
        """
        Overload the or operator to create an OrAspect as part of the tree structure.

        :param other: The other aspect to combine with
        :return: An OrAspect containing both, this and the other, aspect
        """
        return OrAspect([self, other])

    def __neg__(self):
        """
        Overload the not operator to create a NotAspect as part of the tree structure.

        :return: A NotAspect containing this aspect
        """
        return NotAspect(self)

    def __invert__(self):
        """
        Overload the invert operator to create a NotAspect as part of the tree structure.

        :return: A NotAspect containing this aspect
        """
        return NotAspect(self)

    def manage_io(self, func: Callable, *args, **kwargs) -> bool:
        """
        Manages the input and output variables across the whole aspect tree. If the aspect has an input variable, the
        value of this variable will be taken from the variables dictionary of the root node. If an output is defined,
        the result of the function will be stored in the variables dictionary of the root node.

        :param func: Aspect function to call
        :param args: args to pass to the function
        :param kwargs: keyword args to pass to the function
        :return: result of the function
        """
        if self.input:
            if self.input not in self.root.variables.keys():
                raise AttributeError(f"Variable {self.input} not found in variables")
            input_var = self.root.variables[self.input]
            result = func(input_var)
            if self.output:
                self.variables[self.output] = result
        elif self.output:
            result = func(*args, **kwargs)
            self.variables[self.output] = result
            return result
        return func(*args, **kwargs)


class AspectOperator(Aspect):
    """
    Parent class for logical operators to combine multiple aspects in a tree structure. This class adds methods to
    use Aspects as part of a tree structure. Furthermore, there is a method to simplify the tree structure by merging
    Nodes of the same type.
    """
    aspects: List[Aspect]

    def __init__(self, aspects: List[Aspect]):
        """
        Initialize the AspectOperator with a list of aspects. The parent of this node is None, therefore the node is
        always the root of the tree.

        :param aspects: A list of aspects to which are the children of this node
        """
        super().__init__(parent=None, children=aspects)
        self.aspects = aspects
        # self.resolved_aspects = []

    def simplify(self):
        """
        Simplifies the tree structure by merging nodes of the same type. This is done by iterating over the tree and
        merging nodes of the same type.

        :return: Returns the root node of the tree
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


class AndAspect(AspectOperator):
    """
    Class to represent a logical and operator in a tree structure. This class inherits from AspectOperator and adds a
    method to evaluate the children as an and operator.
    """

    def __init__(self, aspects: List[Aspect]):
        """
        Initialize the AndAspect with a list of aspects as the children of this node. This node will be the root of the
        tree.

        :param aspects: A list of aspects which are the children of this node
        """
        super().__init__(aspects)
        self.simplify()

    def __call__(self, *args, **kwargs) -> bool:
        """
        Evaluate the children of this node as an and operator. This is done by iterating over the children and calling
        them with the given arguments. If one child returns False, the evaluation will be stopped and False will be
        returned.

        :param args: A list of arguments to pass to the children
        :param kwargs: A dictionary of keyword arguments to pass to the children
        :return: True if all children return True, False otherwise
        """
        result = True
        for child in self.children:
            # Child is an Aspect and the resolved function should be called
            if child.is_leaf:
                result = result and child(*args, **kwargs)
            # Child is an AspectOperator
            else:
                child(*args, **kwargs)
            if not result:
                return False
        return result


class OrAspect(AspectOperator):
    """
    Class to represent a logical or operator in a tree structure. This class inherits from AspectOperator and adds a
    method to evaluate the children as an or operator.
    """

    def __init__(self, aspects: List[Aspect]):
        """
        Initialize the OrAspect with a list of aspects as the children of this node. This node will be the root of the
        tree.

        :param aspects: A list of aspects which are the children of this node
        """
        super().__init__(aspects)
        self.simplify()

    def __call__(self, *args, **kwargs) -> bool:
        """
        Evaluate the children of this node as an or operator. This is done by iterating over the children and calling
        them with the given arguments. If one child returns True, the evaluation will be stopped and True will be
        returned.

        :param args: A list of arguments to pass to the children
        :param kwargs: A dictionary of keyword arguments to pass to the children
        :return: True if one child returns True, False otherwise
        """
        result = False
        for child in self.children:
            # Child is an Aspect and the resolved function should be called
            if child.is_leaf:
                result = result or child(*args, **kwargs)
            # Child is an AspectOperator
            else:
                result = child(*args, **kwargs)
            if result:
                return True
        return result


class NotAspect(AspectOperator):
    """
    Class to represent a logical not operator in a tree structure. This class inherits from AspectOperator and adds a
    method to evaluate the children as a not operator.
    """

    def __init__(self, aspect: Aspect):
        """
        Initialize the NotAspect with an aspect as the child of this node. This node will be the root of the tree.

        :param aspect: The aspect which is the child of this node
        """
        super().__init__([aspect])

    def __call__(self, *args, **kwargs) -> bool:
        """
        Evaluate the child of this node as a not operator. This is done by calling the child with the given arguments
        and returning the negation of the result.

        :param args: A list of arguments to pass to the child
        :param kwargs: A dictionary of keyword arguments to pass to the child
        :return: The negation of the result of the child
        """
        return not self.children[0](*args, **kwargs)


class ReachableAspect(Aspect):

    def __init__(self, pose: Pose, input: str = None, output: str = None):
        super().__init__(None, None, input, output)
        self.target_pose = pose

    def reachable(self, pose: Pose) -> bool:
        raise NotImplementedError

    def __call__(self, *args, **kwargs):
        return self.manage_io(self.resolved_aspect_function, self.target_pose)


class GraspableAspect(Aspect):

    def __init__(self, object_designator: ObjectDesignatorDescription, input: str = None, output: str = None):
        super().__init__(None, None, input, output)
        self.object_designator = object_designator

    @abstractmethod
    def graspable(self, obj: Object) -> bool:
        raise NotImplementedError

    def __call__(self, *args, **kwargs):
        return self.manage_io(self.resolved_aspect_function, self.object_designator)


class SpaceIsFreeAspect(Aspect):

    def __init__(self, object_designator: ObjectDesignatorDescription, input: str = None, output: str = None):
        super().__init__(None, None, input, output)
        self.object_designator = object_designator

    @abstractmethod
    def space_is_free(self, pose: Pose) -> bool:
        raise NotImplementedError

    def __call__(self, *args, **kwargs):
        return self.manage_io(self.resolved_aspect_function, self.object_designator)


class GripperIsFreeAspect(Aspect):

    def __init__(self, input: str = None, output: str = None):
        super().__init__(None, None, input, output)

    @abstractmethod
    def gripper_is_free(self, gripper: Arms) -> bool:
        raise NotImplementedError

    def __call__(self, *args, **kwargs):
        return self.manage_io(self.resolved_aspect_function)


class VisibleAspect(Aspect):

    def __init__(self, object_designator: ObjectDesignatorDescription, input: str = None, output: str = None):
        super().__init__(None, None, input, output)
        self.object_designator = object_designator

    @abstractmethod
    def is_visible(self, obj: Object) -> bool:
        raise NotImplementedError

    def __call__(self, *args, **kwargs):
        return self.manage_io(self.resolved_aspect_function, self.object_designator)

