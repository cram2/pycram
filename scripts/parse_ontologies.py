import os
from owlready2 import *
from pycrap.parser import OntologiesParser
import matplotlib.pyplot as plt
import networkx as nx

"""
This file parses all relevant ontologies and generates the corresponding Python classes in the pycrap/ontologies folder.
This script will overwrite the existing folders with the same name as the ontologies. Hence, make sure to extract
relevant changes before running this script.
"""


def main():
    ontologies = [
        get_ontology("http://www.ease-crc.org/ont/SOMA.owl").load(),
        # get_ontology("https://raw.githubusercontent.com/hawkina/soma/refs/heads/soma-cram/owl/CROMA.owl").load(),
        # get_ontology("https://raw.githubusercontent.com/hawkina/suturo_knowledge/refs/heads/neems/suturo_knowledge/owl/suturo.owl").load(),
        # get_ontology("https://raw.githubusercontent.com/knowrob/knowrob/refs/heads/dev/owl/URDF.owl").load()
    ]

    path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "..", "src", "pycrap", "ontologies"
    )

    parser = OntologiesParser(ontologies, path)
    nx.draw(
        parser.dependency_graph,
        labels={node: node.name for node in parser.dependency_graph.nodes()},
    )
    plt.show()
    assert nx.is_directed_acyclic_graph(parser.dependency_graph)
    parser.parse()


if __name__ == "__main__":
    main()
