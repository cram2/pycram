from __future__ import annotations

import typing_extensions
from typing_extensions import Type, Optional

from pycrap.urdf_parser import parse_furniture
from pycrap.ontologies import Base, PhysicalObject

if typing_extensions.TYPE_CHECKING:
    from .world import World


class HasConcept:
    """
    A mixin class that adds an ontological concept and individual to a class that will be registered in PyCRAP.
    """

    ontology_concept: Type[Base] = Base
    """
    The ontological concept that this class represents.
    """

    ontology_individual: Optional[Base] = None
    """
    The individual in the ontology that is connected with this class.
    """

    onto_name: Optional[str] = None
    """
    The name of the individual in the ontology.
    """

    def __init__(self, world: World, name: Optional[str] = None, concept: Type[Base] = PhysicalObject,
                 parse_name: bool = True):
        self.onto_name = name
        self.ontology_concept = concept

        if world.is_prospection_world:
            return

        self.ontology_individual = self.ontology_concept(name=name.lower() if name is not None else None,
                                                         namespace=world.ontology.ontology)
        if parse_name:
            inferred_concept = parse_furniture(name)
            if inferred_concept is not None:
                self.ontology_individual.is_a = [inferred_concept]

        world.ontology.python_objects[self.ontology_individual] = self
