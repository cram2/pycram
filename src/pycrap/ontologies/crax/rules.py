from __future__ import annotations

from abc import abstractmethod, ABC

from owlready2 import Imp, Ontology
from typing_extensions import Type, Dict

from .object_properties import ontology as CRAXOntology, is_part_of, contains_object, CRAX_ONTOLOGY_NAME


class CRAXRule(ABC):
    """
    A class that represents a rule in PyCRAP.
    """
    all_rules: Dict[Ontology, Dict[Type[CRAXRule], CRAXRule]] = {}
    """
    A dictionary that maps ontology rule classes to rule instances, so that the same rule is not created multiple times.
    """

    def __init__(self, ontology: Ontology = CRAXOntology,
                 ontology_name: str = CRAX_ONTOLOGY_NAME):
        """
        Creates a new CRAXRule object and sets the rule in the ontology.

        :param ontology: The ontology to add the rule to.
        :param ontology_name: The name of the ontology.
        """
        self.ontology = ontology
        self.ontology_name = ontology_name
        if self.rule_in_all_rules:
            self.rule = self._get_old_rule()
        else:
            self.rule = self._add_rule()

    @property
    def rule_in_all_rules(self):
        return self.ontology in CRAXRule.all_rules and type(self) in CRAXRule.all_rules[self.ontology]

    def _get_old_rule(self):
        """
        Gets the old rule from the all_rules dictionary.
        """
        return CRAXRule.all_rules[self.ontology][type(self)].rule

    def _add_rule(self):
        """
        Adds the rule to the ontology.
        """
        rule = Imp(namespace=self.ontology)
        rule_body = self.rule_body.replace(f"{self.ontology_name}.", "")
        rule.set_as_rule(rule_body)
        return rule

    @property
    def name(self):
        """
        :return: The name of the rule
        """
        return self.__class__.__name__

    @property
    @abstractmethod
    def rule_body(self):
        """
        :return: The body of the rule as a string in SWRL syntax.
        """
        pass


class HierarchicalContainment(CRAXRule):
    """
    A rule that asserts that parent objects contain the objects contained by their parts.
    """

    @property
    def rule_body(self):
        return f"""
        {is_part_of}(?part, ?parent), {contains_object}(?part, ?object)
        -> {contains_object}(?parent, ?object)
        """
