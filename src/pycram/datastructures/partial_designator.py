# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from collections import Iterable
from typing_extensions import Type, List, Tuple, Any, Dict, TYPE_CHECKING
from itertools import product
from inspect import signature


if TYPE_CHECKING:
    from ..designator import ActionDesignatorDescription


class PartialDesignator:
    """
    A partial designator_description is somewhat between a DesignatorDescription and a specified designator_description. Basically it is a
    partially initialized specified designator_description which can take a list of input arguments (like a DesignatorDescription)
    and generate a list of specified designators with all possible permutations of the input arguments.

    PartialDesignators are designed as generators, as such they need to be iterated over to yield the possible specified
    designators. Please also keep in mind that at the time of iteration all parameter of the specified designator_description need
    to be filled, otherwise a TypeError will be raised, see the example below for usage.

    .. code-block:: python
            # Example usage
            partial_designator = PartialDesignator(PickUpActionPerformable, milk_object_designator, arm=[Arms.RIGHT, Arms.LEFT])
            for performable in partial_designator(Grasp.FRONT):
                performable.perform()
    """
    performable: Type[ActionDesignatorDescription.Action]
    """
    Reference to the performable class that should be initialized
    """
    args: Tuple[Any, ...]
    """
    Arguments that are passed to the performable
    """
    kwargs: Dict[str, Any]
    """
    Keyword arguments that are passed to the performable
    """

    def __init__(self, performable: Type[ActionDesignatorDescription.Action], *args, **kwargs):
        self.performable = performable
        self.args = args
        self.kwargs = kwargs

    def __call__(self, *fargs, **fkwargs):
        """
        Creats a new PartialDesignator with the given arguments and keyword arguments added.

        :param fargs: Additional arguments that should be added to the new PartialDesignator
        :param fkwargs: Additional keyword arguments that should be added to the new PartialDesignator
        :return: A new PartialDesignator with the given arguments and keyword arguments added
        """
        newkeywords = {**self.kwargs, **fkwargs}
        return PartialDesignator(self.performable, *self.args, *fargs, **newkeywords)

    def __iter__(self) -> Type[ActionDesignatorDescription.Action]:
        """
        Iterates over all possible permutations of the arguments and keyword arguments and creates a new performable
        object for each permutation.

        :return: A new performable object for each permutation of arguments and keyword arguments
        """
        for args_combination in PartialDesignator.generate_permutations(self.args):
            for kwargs_combination in PartialDesignator.generate_permutations(self.kwargs.values()):
                yield self.performable(*args_combination, **dict(zip(self.kwargs.keys(), kwargs_combination)))

    @staticmethod
    def generate_permutations(args: Iterable) -> List:
        """
        Generates all possible permutations of the given arguments. This uses itertools.product to generate the
        permutations.

        :param args: An iterable with arguments
        :yields: A list with a possible permutation of the given arguments
        """
        iter_list = [x if PartialDesignator._is_iterable(x) else [x] for x in args]
        for combination in product(*iter_list):
            yield combination

    @staticmethod
    def _is_iterable(obj: Any) -> bool:
        """
        Checks if the given object is iterable.

        :param obj: The object that should be checked
        :return: True if the object is iterable, False otherwise
        """
        try:
            iter(obj)
        except TypeError:
            return False
        return True

    def missing_parameter(self) -> List[str]:
        """
        Returns a list of all parameters that are missing for the performable to be initialized.

        :return: A list of parameter names that are missing from the performable
        """
        performable_params = signature(self.performable).parameters
        # List of all parameter names that need to be filled for the performable
        param_names = list(performable_params.keys())

        # Remove parameter that are filled by args
        missing_after_args = param_names[len(self.args):]
        # Remove parameter that are filled by keyword arguments and return the remaining parameters
        return list(set(missing_after_args) - set(self.kwargs.keys()))


