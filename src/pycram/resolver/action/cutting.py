import dataclasses

from ...designators.action_designator import (CuttingAction)
import os
import subprocess

from ...bullet_world import BulletWorld
from ...robot_descriptions import robot_description
from ...task import with_tree
from ... import helper
from ...designators.motion_designator import *

from rdflib import Graph, Literal, URIRef, Namespace
from rdflib.namespace import OWL, RDF, RDFS


class CuttingActionSPARQL(CuttingAction):

    def __init__(self, object_to_be_cut: ObjectDesignatorDescription, tool: ObjectDesignatorDescription, arms: List[str]):
        super().__init__(object_to_be_cut, tool, arms)
        output = subprocess.check_output(["rospack", "find", "pycram"], universal_newlines=True).strip()
        self.query_folder: str = output + "/src/pycram/resolver/action/queries/"
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

    def __iter__(self):
        for action in iter(CuttingAction.__iter__(self)):
            action.technique = "slicing"
            action.slice_thickness = 3

            yield action

    def cutting_from_sparql(self) -> CuttingAction.Action:
        return self.Action(self.object_to_be_cut.ground(), self.tool.ground(), self.arms[0])
