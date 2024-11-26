import tempfile

import owlready2
from typing_extensions import Optional


class Ontology:
    """
    Wrapper class for user-friendly access of the owlready2 ontology class.
    """

    ontology: owlready2.Ontology
    """
    The owlready2 ontology that is used for reasoning.
    """

    file: tempfile.NamedTemporaryFile
    """
    The file that the ontology is stored in.
    """


    def __init__(self):
        self.file = tempfile.NamedTemporaryFile(delete=True)
        self.ontology = owlready2.get_ontology("file://" + self.path).load()
        self.ontology.name = "PyCRAP_" + str(id(self))

    @property
    def path(self) -> str:
        return self.file.name

    def individuals(self):
        return self.ontology.individuals()

    def destroy_individuals(self):
        """
        Destroys all individuals in the ontology.
        """
        for individual in self.individuals():
            owlready2.destroy_entity(individual)