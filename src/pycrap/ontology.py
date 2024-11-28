import tempfile

import owlready2
from .base import default_pycrap_ontology


class Ontology:
    """
    Wrapper class for user-friendly access of the owlready2 ontology class.

    This class spawns a temporary file that is used to store the ontology.
    This has to be done whenever several PyCRAP ontologies are needed that store different individuals.
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
        self.ontology.name = "PyCRAP"

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

    @staticmethod
    def classes():
        """
        :return: All classes of the PyCRAP ontology.
        """
        return default_pycrap_ontology.classes()

    def search(self, *args, **kwargs):
        """
        :return: The search results of the ontology.
        """
        return self.ontology.search(*args, **kwargs)

    def __enter__(self):
        return self.ontology.__enter__()

    def __exit__(self, exc_type, exc_val, exc_tb):
        return self.ontology.__exit__(exc_type, exc_val, exc_tb)

    def reason(self):
        """
        Reason over the ontology. This may take a long time.
        """
        owlready2.sync_reasoner([self.ontology, default_pycrap_ontology])