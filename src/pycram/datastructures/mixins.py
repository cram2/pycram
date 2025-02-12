from typing_extensions import Type, Optional

from pycrap import Base


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

    def __init__(self):
        self.ontology_individual = self.ontology_concept()