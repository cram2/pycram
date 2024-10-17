# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from abc import abstractmethod

from .enums import Arms
from .dataclasses import ReasoningResult
from .pose import Pose
from typing_extensions import List, Iterable, Dict, Any, Callable, Type, TYPE_CHECKING
from anytree import NodeMixin, PreOrderIter, Node
from ..failures import ObjectNotVisible, ManipulationPoseUnreachable, NavigationGoalInCollision, ObjectUnfetchable, \
    GripperOccupied, PlanFailure

if TYPE_CHECKING:
    from ..designators.object_designator import ObjectDesignatorDescription

class Property(NodeMixin):
    """
    Parent class to represent a semantic property as part of a knowledge pre-condition of designators.
    Aspects can be combined using logical operators to create complex pre-conditions, the resulting expression is a
    datastructure of a tree.
    """
    variables: Dict[str, Any]
    """
    Dictionary of variables and their values which are used in the property tree. This dictionary is only to be used in 
    the root node.
    """

    def __init__(self, parent: NodeMixin = None, children: Iterable[NodeMixin] = None):
        super().__init__()
        self.parent: Property = parent
        self.variables: Dict[str, Any] = {}
        self.executed: bool = False
        self.result: ReasoningResult = None
        if children:
            self.children = children

    def __and__(self, other):
        """
        Overload the and operator to create an And as part of the tree structure.

        :param other: The other property to combine with
        :return: An And containing both, this and the other, property
        """
        return And([self, other])

    def __or__(self, other):
        """
        Overload the or operator to create an Or as part of the tree structure.

        :param other: The other property to combine with
        :return: An Or containing both, this and the other, property
        """
        return Or([self, other])

    def __neg__(self):
        """
        Overload the not operator to create a Not as part of the tree structure.

        :return: A Not containing this property
        """
        return Not(self)

    def __invert__(self):
        """
        Overload the invert operator to create a Not as part of the tree structure.

        :return: A Not containing this property
        """
        return Not(self)

    def manage_io(self, func: Callable, *args, **kwargs) -> bool:
        """
        Manages the ReasoningResult of a property function and calls it. The success of the method will be passed to the
        parent node while the reasoned parameters will be stored in the variables dictionary of the root node.

        :param func: Property function to call
        :param args: args to pass to the function
        :param kwargs: keyword args to pass to the function
        :return: result of the function
        """
        reasoning_result = func(*args, **kwargs)
        self.root.variables.update(reasoning_result.reasoned_parameter)
        self.result = reasoning_result
        return reasoning_result.success

    @property
    def successful(self) -> bool:
        """
        Returns the result of the complete tree structure. This is done by iterating over the tree and checking if all
        nodes return True.

        :return: True if all nodes in the tree are True, False otherwise
        """
        success = self.result
        for node in PreOrderIter(self.root):
            success &= node.result.success
        return success

    @property
    def resolved(self) -> bool:
        """
        Returns True if all nodes in the tree are resolved. This is done by iterating over the tree and checking if all
        nodes are either a ResolvedProperty or a PropertyOperator.

        :return: True if all nodes in the tree are resolved, False otherwise
        """
        res = True
        for node in PreOrderIter(self.root):
            res &= isinstance(node, (ResolvedProperty, PropertyOperator))
        return res


class PropertyOperator(Property):
    """
    Parent class for logical operators to combine multiple properties in a tree structure. This class adds methods to
    use Properties as part of a tree structure. Furthermore, there is a method to simplify the tree structure by merging
    Nodes of the same type.
    """

    def __init__(self, properties: List[Property]):
        """
        Initialize the PropertyOperator with a list of Properties. The parent of this node is None, therefore the node is
        always the root of the tree.

        :param properties: A list of properties to which are the children of this node
        """
        super().__init__(parent=None, children=properties)
        self.result = True

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


class And(PropertyOperator):
    """
    Class to represent a logical and operator in a tree structure. This class inherits from PropertyOperator and adds a
    method to evaluate the children as an and operator.
    """

    def __init__(self, properties: List[Property]):
        """
        Initialize the And with a list of properties as the children of this node. This node will be the root of the
        tree.

        :param properties: A list of properties which are the children of this node
        """
        super().__init__(properties)
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
            # Child is a Property and the executed function should be called
            if child.is_leaf:
                result = result and child(*args, **kwargs)
            # Child is a PropertyOperator
            else:
                child(*args, **kwargs)
            if not result:
                return False
        return result


class Or(PropertyOperator):
    """
    Class to represent a logical or operator in a tree structure. This class inherits from PropertyOperator and adds a
    method to evaluate the children as an or operator.
    """

    def __init__(self, properties: List[Property]):
        """
        Initialize the Or with a list of properties as the children of this node. This node will be the root of the
        tree.

        :param properties: A list of properties which are the children of this node
        """
        super().__init__(properties)
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
            # Child is a Property and the executed function should be called
            if child.is_leaf:
                result = result or child(*args, **kwargs)
            # Child is a PropertyOperator
            else:
                result = child(*args, **kwargs)
            if result:
                return True
        return result


class Not(PropertyOperator):
    """
    Class to represent a logical not operator in a tree structure. This class inherits from PropertyOperator and adds a
    method to evaluate the children as a not operator.
    """

    def __init__(self, property: Property):
        """
        Initialize the Not with an aspect as the child of this node. This node will be the root of the tree.

        :param property: The property which is the child of this node
        """
        super().__init__([property])

    def __call__(self, *args, **kwargs) -> bool:
        """
        Evaluate the child of this node as a not operator. This is done by calling the child with the given arguments
        and returning the negation of the result.

        :param args: A list of arguments to pass to the child
        :param kwargs: A dictionary of keyword arguments to pass to the child
        :return: The negation of the result of the child
        """
        return not self.children[0](*args, **kwargs)


class ResolvedProperty(Property):
    """
    Class to represent a executed property function. It holds the reference to the respective function in the knowledge
    source and the exception that should be raised if the property is not fulfilled. Its main purpose is to manage the
    call to the property function as well as handle the input and output variables.
    """

    resolved_property_function: Callable
    """
    Reference to the actual implementation of the property function in the KnowledgeSource.
    """
    property_exception: Type[PlanFailure]
    """
    Exception that should be raised if the property is not fulfilled.
    """

    def __init__(self, resolved_property_function: Callable, property_exception: Type[PlanFailure], parent: NodeMixin = None):
        """
        Initialize the ResolvedProperty with the executed property function, the exception that should be raised if the property
        is not fulfilled, the parent node.

        :param resolved_property_function: Reference to the function in the knowledge source
        :param property_exception: Exception that should be raised if the property is not fulfilled
        :param parent: Parent node of this property
        """
        super().__init__(parent, None)
        self.resolved_property_function = resolved_property_function
        self.property_exception = property_exception
        self.parameter = {}

    def __call__(self, *args, **kwargs) -> bool:
        """
        Manages the io of the call to the property function and then calls the function. If the function returns False,
        the exception defined in :attr:`property_exception` will be raised.

        :return: The result of the property function
        """
        result = self.manage_io(self.resolved_property_function, **self.parameter)
        if not result:
            raise self.property_exception(f"Property function {self.resolved_property_function} returned False")
        return result


class ReachableProperty(Property):
    property_exception = ManipulationPoseUnreachable

    def __init__(self, pose: Pose):
        super().__init__(None, None)
        self.pose = pose

    @abstractmethod
    def reachable(self, pose: Pose) -> ReasoningResult:
        raise NotImplementedError


class GraspableProperty(Property):
    property_exception = ObjectUnfetchable

    def __init__(self, object_designator: ObjectDesignatorDescription):
        super().__init__(None, None)
        self.object_designator = object_designator

    @abstractmethod
    def graspable(self, object_designator: ObjectDesignatorDescription) -> ReasoningResult:
        raise NotImplementedError


class SpaceIsFreeProperty(Property):
    property_exception = NavigationGoalInCollision

    def __init__(self, pose: Pose):
        super().__init__(None, None)
        self.pose = pose

    @abstractmethod
    def space_is_free(self, pose: Pose) -> ReasoningResult:
        raise NotImplementedError


class GripperIsFreeProperty(Property):
    property_exception = GripperOccupied

    def __init__(self, gripper: Arms):
        super().__init__(None, None)
        self.gripper = gripper

    @abstractmethod
    def gripper_is_free(self, gripper: Arms) -> ReasoningResult:
        raise NotImplementedError


class VisibleProperty(Property):
    property_exception = ObjectNotVisible

    def __init__(self, object_designator: ObjectDesignatorDescription):
        super().__init__(None, None)
        self.object_designator = object_designator

    @abstractmethod
    def is_visible(self, object_designator: ObjectDesignatorDescription) -> ReasoningResult:
        raise NotImplementedError
