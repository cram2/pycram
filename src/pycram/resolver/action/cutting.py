import dataclasses

from ...designators.action_designator import (CuttingAction)
import os

from ...bullet_world import BulletWorld
from ...robot_descriptions import robot_description
from ...task import with_tree
from ... import helper
from ...designators.motion_designator import *

from rdflib import Graph, Literal, URIRef, Namespace
from rdflib.namespace import OWL, RDF, RDFS


class CuttingActionSPARQL(CuttingAction):

    def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[str], grasps: List[str]):
        super().__init__(object_designator_description, arms, grasps, [0])
        self.query_folder: str = os.path.join(os.path.expanduser("~", ), "pycram_ws", "src", "pycram", "demos",
                                              "pycram_possible_actions_demo", "queries")
        task = "cut:Quartering"
        tobject = "obo:FOODON_03301710"

        self.knowledge_graph = Graph()

        # define prefixes to be used in the query
        SOMA = Namespace("http://www.ease-crc.org/ont/SOMA.owl#")
        CUT2 = Namespace("http://www.ease-crc.org/ont/situation_awareness#")
        CUT = Namespace("http://www.ease-crc.org/ont/food_cutting#")
        DUL = Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")
        OBO = Namespace("http://purl.obolibrary.org/obo/")
        self.knowledge_graph.bind("owl", OWL)
        self.knowledge_graph.bind("rdfs", RDFS)
        self.knowledge_graph.bind("soma", SOMA)
        self.knowledge_graph.bind("cut2", CUT2)
        self.knowledge_graph.bind("cut", CUT)
        self.knowledge_graph.bind("dul", DUL)
        self.knowledge_graph.bind("obo", OBO)

    @property
    def repetitions(self):
        task = "cut:Quartering"
        tobject = "obo:FOODON_03301710"
        repetitionsquery = """  SELECT ?rep WHERE {
              SERVICE <https://api.krr.triply.cc/datasets/mkumpel/FruitCuttingKG/services/FoodCuttingKG/sparql> {  
          {
             OPTIONAL{ %s rdfs:subClassOf ?action}
                ?action rdfs:subClassOf* ?rep_node.
                ?rep_node owl:onProperty cut:repetitions.
                FILTER EXISTS {
                    ?rep_node owl:hasValue ?val.}
                BIND("0.05" AS ?rep)}
            UNION
            {
               OPTIONAL{ %s rdfs:subClassOf ?action }
                ?action rdfs:subClassOf* ?rep_node.
                ?rep_node owl:onProperty cut:repetitions.
                FILTER EXISTS {
                    ?rep_node owl:minQualifiedCardinality ?val.}
                BIND("more than 1" AS ?rep)}} }""" % (task, task)
        for row in self.knowledge_graph.query(repetitionsquery):
            return row.rep

    @property
    def technique(self):
        task = "cut:Quartering"
        tobject = "obo:FOODON_03301710"
        positionquery = """ SELECT ?pos WHERE {
        SERVICE <https://api.krr.triply.cc/datasets/mkumpel/FruitCuttingKG/services/FoodCuttingKG/sparql> {
        OPTIONAL { %s rdfs:subClassOf ?sub.}
        ?sub rdfs:subClassOf* ?node.
        ?node owl:onProperty cut:affordsPosition.
        ?node owl:someValuesFrom ?position.
        BIND(IF(?position = cut:halving_position, "halving", "slicing") AS ?pos)
        } }""" % (task)
        for row in self.knowledge_graph.query(positionquery):
            return row.pos

    def __iter__(self):
        for action in iter(CuttingAction.__iter__(self)):
            action.technique = 'slicing'
            action.slice_thickness = float(self.repetitions)

            yield action

    def cutting_from_sparql(self) -> CuttingAction.Action:
        return self.Action(self.object_designator_description.ground(), self.arms[0], self.grasps[0])
