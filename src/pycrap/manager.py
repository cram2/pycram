import tempfile
from typing import Optional

from owlready2 import Ontology, get_ontology


class OntologyManager:

    ontology: Optional[Ontology] = None
    crax_path: str = None

    def __init__(self, crax_path: Optional[str] = None):

        if crax_path:
            self.crax_path = crax_path

        if self.crax_path is None:
            temp_file = tempfile.NamedTemporaryFile(delete=True)
            self.crax_path = temp_file.name

        self.ontology = get_ontology("file://" + self.crax_path).load()
        self.ontology.name = "PyCRAP"