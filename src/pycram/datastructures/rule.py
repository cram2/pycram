from __future__ import annotations
import typing_extensions
from owlready2 import Imp

from pycram.datastructures.mixins import HasConcept

if typing_extensions.TYPE_CHECKING:
    from pycram.datastructures.world import World
    from pycrap.ontologies.crax.rules import CRAXRule


class Rule(HasConcept):
    """
    A class that represents a rule in PyCRAP.
    """

    def __init__(self, crax_rule: CRAXRule, world: World):
        """
        Creates a new Rule object.

        :param crax_rule: The CRAXRule to add/assert which uses SWRL syntax.
        :param world: The world that the Rule object is in.
        """
        super().__init__(world, crax_rule.name, concept=Imp, parse_name=False)
        rule = crax_rule.rule.replace(f"{world.ontology.ontology.name}.", "")
        self.ontology_individual.set_as_rule(rule)
