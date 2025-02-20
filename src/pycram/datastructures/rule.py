from __future__ import annotations
import typing_extensions
from owlready2 import Imp

from pycram.datastructures.mixins import HasConcept

if typing_extensions.TYPE_CHECKING:
    from pycram.datastructures.world import World


class Rule(HasConcept):
    """
    A class that represents a rule in PyCRAP.
    """

    def __init__(self, rule: str, world: World, name: str):
        """
        Creates a new Rule object.

        :param rule: The rule that the Rule object represents which uses SWRL syntax.
        :param world: The world that the Rule object is in.
        :param name: The name of the Rule object.
        """
        super().__init__(world, name, concept=Imp, parse_name=False)
        rule = rule.replace(f"{world.ontology.ontology.name}.", "")
        self.ontology_individual.set_as_rule(rule)
