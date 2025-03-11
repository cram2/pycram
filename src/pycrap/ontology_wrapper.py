import tempfile

import owlready2
from typing_extensions import Dict, Any

from .ontologies.base import Base, ontology as default_pycrap_ontology, ontology_file as default_pycrap_ontology_file, \
    CRAX_ONTOLOGY_NAME


class OntologyWrapper:
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

    python_objects: Dict[Base, Any]
    """
    A dictionary that maps ontology individuals to python objects.
    """

    def __init__(self):
        self.file = default_pycrap_ontology_file
        self.ontology = default_pycrap_ontology
        self.ontology.name = CRAX_ONTOLOGY_NAME
        self.python_objects = {}

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
        Check https://owlready2.readthedocs.io/en/latest/onto.html#simple-queries for details.

        :return: The search results of the ontology.
        """
        return self.ontology.search(*args, **kwargs)

    def search_one(self, *args, **kwargs):
        return self.ontology.search(*args, **kwargs)

    def __enter__(self):
        return self.ontology.__enter__()

    def __exit__(self, exc_type, exc_val, exc_tb):
        return self.ontology.__exit__(exc_type, exc_val, exc_tb)

    def reason(self):
        """
        Reason over the ontology. This may take a long time.
        """
        owlready2.sync_reasoner(self.ontology, infer_property_values=True, debug=False)

    def add_individual(self, individual: Base, python_object: Any):
        self.python_objects[individual] = python_object
