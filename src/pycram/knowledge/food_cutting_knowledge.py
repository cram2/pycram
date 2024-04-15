from ..datastructures.knowledge_source import KnowledgeSource

from rdflib import Graph, Namespace
from rdflib.namespace import OWL, RDFS

from urllib import request
from typing_extensions import Iterable

class FoodCuttingKnowledge(KnowledgeSource):

    def __init__(self):
        super().__init__("Food KG", 1)
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
    def is_available(self) -> bool:
        return request.urlopen("https://api.krr.triply.cc/datasets/mkumpel/FruitCuttingKG/services/FoodCuttingKG/sparql?query=SELECT%20?subject%20?predicate%20?objectWHERE%20{?subject%20?predicate%20?object%20.}").getcode() != 404

    @property
    def is_connected(self) -> bool:
        return self.is_available

    def connect(self):
        pass

    def query(self, designator):
        pass

    def get_repetitions(self, task, task_object) -> Iterable[str]:
        # task = "cut:Quartering"
        # tobject = "obo:FOODON_03301710"
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
            yield row.rep

    def get_technique_for_task(self, task, task_object) -> Iterable[str]:
        # task = "cut:Quartering"
        # tobject = "obo:FOODON_03301710"
        positionquery = """ SELECT ?pos WHERE {
           SERVICE <https://api.krr.triply.cc/datasets/mkumpel/FruitCuttingKG/services/FoodCuttingKG/sparql> {
           OPTIONAL { %s rdfs:subClassOf ?sub.}
           ?sub rdfs:subClassOf* ?node.
           ?node owl:onProperty cut:affordsPosition.
           ?node owl:someValuesFrom ?position.
           BIND(IF(?position = cut:halving_position, "halving", "slicing") AS ?pos)
           } }""" % task
        for row in self.knowledge_graph.query(positionquery):
            yield row.pos
