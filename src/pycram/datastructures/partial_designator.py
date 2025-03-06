# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from typing_extensions import Type, List, Tuple, Any, Dict, TYPE_CHECKING, TypeVar, Generic, Iterator, Iterable
from itertools import product
from inspect import signature

from ..utils import is_iterable


if TYPE_CHECKING:
    from ..designator import ActionDescription

    T = TypeVar('T', bound=Type[ActionDescription])
    Supertype = Iterable[ActionDescription]
else:
    Supertype = Iterable

class PartialDesignator(Supertype):
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

    def __init__(self, performable: T, *args, **kwargs):
        self.performable = performable
        self.kwargs = dict(signature(self.performable).bind_partial(*args, **kwargs).arguments)
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
        #return PartialDesignator(self.performable, *fargs, **newkeywors)

    def __iter__(self) -> T:
        """
        Iterates over all possible permutations of the arguments and keyword arguments and creates a new performable
        object for each permutation. In case there are conflicting parameters the args will be used over the keyword
        arguments.

        :return: A new performable object for each permutation of arguments and keyword arguments
        """
        for kwargs_combination in self.generate_permutations():
            yield self.performable(**dict(zip(self.kwargs.keys(), kwargs_combination)))

    def generate_permutations(self) -> List:
        """
        Generates the cartesian product of the given arguments. Arguments can also be a list of lists of arguments.

        :yields: A list with a possible permutation of the given arguments
        """
        iter_list = [x if is_iterable(x) else [x] for x in self.kwargs.values()]
        for combination in product(*iter_list):
            yield combination

    def missing_parameter(self) -> List[str]:
        """
        Returns a list of all parameters that are missing for the performable to be initialized.

        :return: A list of parameter names that are missing from the performable
        """
        missing = {k: v for k, v in self.kwargs.items() if v is None}
        return list(missing.keys())

    def resolve(self):
        return next(iter(self))



