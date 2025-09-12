# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from typing_extensions import Type, List, Tuple, Any, Dict, TYPE_CHECKING, TypeVar, Generic, Iterator, Iterable, AnyStr
from inspect import signature

# from ..designator import DesignatorDescription
from ..has_parameters import leaf_types, HasParameters
from ..plan import PlanNode
from ..utils import is_iterable, lazy_product

T = TypeVar('T')

class PartialDesignator(Iterable[T]):
    """
    A partial designator_description is somewhat between a DesignatorDescription and a specified designator_description. Basically it is a
    partially initialized specified designator_description which can take a list of input arguments (like a DesignatorDescription)
    and generate a list of specified designators with all possible permutations of the input arguments.

    PartialDesignators are designed as generators, as such they need to be iterated over to yield the possible specified
    designators. Please also keep in mind that at the time of iteration all parameter of the specified designator_description need
    to be filled, otherwise a TypeError will be raised, see the example below for usage.

    .. code-block:: python

            # Example usage
            partial_designator = PartialDesignator(PickUpAction, milk_object_designator, arm=[Arms.RIGHT, Arms.LEFT])
            for performable in partial_designator(Grasp.FRONT):
                performable.perform()
    """
    performable: T = None
    """
    Reference to the performable class that should be initialized
    """
    args: Tuple[Any, ...] = None
    """
    Arguments that are passed to the performable
    """
    kwargs: Dict[str, Any] = None
    """
    Keyword arguments that are passed to the performable
    """

    _plan_node: PlanNode = None
    """
    Reference to the PlanNode that is used to execute the performable
    """

    def __init__(self, performable: T, *args, **kwargs):
        self.performable = performable
        # We use the init of the performable class since typing for the whole class messes up the signature of the class.
        # This is not optimal since "self" needs to be filtered from the parameter list but it works.
        sig = signature(self.performable.__init__)
        params_without_self = list(filter(None, [param if name != "self" else None for name, param in sig.parameters.items()]))
        sig_without_self = sig.replace(parameters=params_without_self)
        self.kwargs = dict(sig_without_self.bind_partial(*args, **kwargs).arguments)
        for key in dict(signature(self.performable).parameters).keys():
            if key not in self.kwargs.keys():
                self.kwargs[key] = None

    def __call__(self, *fargs, **fkwargs):
        """
        Creates a new PartialDesignator with the given arguments and keyword arguments added. Existing arguments will
        be prioritized over the new arguments.

        :param fargs: Additional arguments that should be added to the new PartialDesignator
        :param fkwargs: Additional keyword arguments that should be added to the new PartialDesignator
        :return: A new PartialDesignator with the given arguments and keyword arguments added
        """
        newkeywors = {k: v for k, v in self.kwargs.items() if v is not None}
        for key, value in fkwargs.items():
            if key in newkeywors.keys() and newkeywors[key] is None:
                newkeywors[key] = value
            elif key not in newkeywors.keys():
                newkeywors[key] = value
        self.kwargs.update(dict(signature(self.performable).bind_partial(*fargs, **fkwargs).arguments))
        return self

    def __iter__(self) -> Iterator[T]:
        """
        Iterates over all possible permutations of the arguments and keyword arguments and creates a new performable
        object for each permutation. In case there are conflicting parameters the args will be used over the keyword
        arguments.

        :return: A new performable object for each permutation of arguments and keyword arguments
        """
        for kwargs_combination in self.generate_permutations():
            yield self.performable(**kwargs_combination)

    def generate_permutations(self) -> Iterator[Dict[str, Any]]:
        """
        Generates the cartesian product of the given arguments. Arguments can also be a list of lists of arguments.

        :yields: A list with a possible permutation of the given arguments
        """
        iter_list = [x if is_iterable(x) and not type(x) == str else [x] for x in self.kwargs.values()]
        for combination in lazy_product(*iter_list, iter_names=list(self.kwargs.keys())):
            yield dict(zip(self.kwargs.keys(), combination))

    def missing_parameter(self) -> List[str]:
        """
        Returns a list of all parameters that are missing for the performable to be initialized.

        :return: A list of parameter names that are missing from the performable
        """
        missing = {k: v for k, v in self.kwargs.items() if v is None}
        return list(missing.keys())

    def resolve(self) -> T():
        """
        Returns the Designator with the first set of parameters

        :return: A fully parametrized Designator
        """
        return next(iter(self))

    def to_dict(self):
        desig = {"type": self.performable.__name__}
        desig.update(self.kwargs)
        return desig

    def flatten(self) -> List[leaf_types]:
        """
        Flattens a partial designator, very similar to HasParameters.flatten but this method can deal with parameters
        thet are None.

        :return: A list of flattened field values from the object.
        """
        result = []
        for field_name, field_type in self.performable._parameters.items():
            if self.kwargs[field_name] is not None:
                if issubclass(field_type, HasParameters):
                    sub_obj = self.kwargs[field_name]
                    result.extend(sub_obj.flatten())
                else:
                    result.append(self.kwargs[field_name])
            else:
                result.append(self.kwargs[field_name])
        return result

    def flatten_parameters(self) -> Dict[str, leaf_types]:
        """
        The flattened parameter types of the performable.

        :return: A dict with the flattened parameter types of the performable.
        """
        return self.performable.flatten_parameters()

    @property
    def plan_node(self) -> PlanNode:
        """
        Returns the PlanNode that is used to execute the performable.

        :return: The PlanNode that is used to execute the performable.
        """
        return self._plan_node

    @plan_node.setter
    def plan_node(self, value: PlanNode):
        """
        Sets the PlanNode that is used to execute the performable.

        :param value: The PlanNode that is used to execute the performable.
        """
        if not isinstance(value, PlanNode):
            raise TypeError("plan_node must be an instance of PlanNode")
        self._plan_node = value
        for key, value in self.kwargs.items():
            if "DesignatorDescription" in [c.__name__ for c in value.__class__.__mro__]:
                value.plan_node = self._plan_node





