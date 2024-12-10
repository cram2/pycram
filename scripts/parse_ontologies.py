import os
from owlready2 import *
from pycrap.parser import OntologiesParser

"""
This file parses all relevant ontologies and generates the corresponding Python classes in the pycrap/ontologies folder.
This script will overwrite the existing folders with the same name as the ontologies. Hence, make sure to extract
relevant changes before running this script.
"""

def main():
    ontologies = [
        get_ontology("http://www.ease-crc.org/ont/SOMA.owl").load()
    ]

    path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "src", "pycrap", "ontologies")

    parser = OntologiesParser(ontologies, path)
    parser.parse()


if __name__ == "__main__":
    main()