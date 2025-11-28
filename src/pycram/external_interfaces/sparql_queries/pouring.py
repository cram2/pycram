from SPARQLWrapper import SPARQLWrapper, JSON

sparql = SPARQLWrapper(
    "https://knowledgedb.informatik.uni-bremen.de/mealprepDB/MealPreparation/query"
)
sparql.setReturnFormat(JSON)

prefix = """
 PREFIX owl: <http://www.w3.org/2002/07/owl#>
 prefix dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#>
 PREFIX pour: <http://www.ease-crc.org/ont/meals#>
 PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
 PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
 PREFIX foodon: <http://purl.obolibrary.org/obo/>
 PREFIX soma: <http://www.ease-crc.org/ont/SOMA.owl#>
 PREFIX sit_aware: <http://www.ease-crc.org/ont/situation_awareness#>
 PREFIX obo: <http://purl.obolibrary.org/obo/>
 PREFIX qudt: <http://qudt.org/schema/qudt#>
 """


def get_needed_tool(verb):
    query = """
     SELECT ?res WHERE {
                        %s rdfs:subClassOf ?sub.
                        ?sub owl:onProperty dul:hasParticipant.
                        ?sub owl:someValuesFrom ?neededtool.
                        BIND(REPLACE(STR(?neededtool), "^.*[#/]", "") AS ?res).
                    }
    """ % (
        verb
    )
    full_query = prefix + query
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return (
        results["results"]["bindings"][0]["res"]["value"]
        if results["results"]["bindings"]
        else None
    )


def get_min_angle(foodobject):
    query = """
    SELECT ?res WHERE {
                        %s rdfs:subClassOf ?cons_node.
                        ?cons_node owl:onProperty pour:hasConsistency.
			?cons_node owl:someValuesFrom ?consistency.
                        ?union rdf:first*/rdf:rest* ?consistency.
 			 {
 			 ?rest rdf:first*/rdf:rest* ?union.
  			?start owl:unionOf ?rest.
 			 ?a owl:someValuesFrom ?start.
 			 }
 			 UNION
 			 {
 			   ?a owl:allValuesFrom ?union.
 			 }
 			 ?f rdf:first*/rdf:rest* ?a.
			  ?r rdf:first*/rdf:rest* ?f.
			  ?i owl:intersectionOf ?r.
			  ?inter owl:someValuesFrom ?i.
			  ?param rdfs:subClassOf ?inter.
			  ?param rdfs:subClassOf pour:PouringAngle.
			  ?param rdfs:subClassOf ?degree1.
			  ?degree1 owl:onProperty qudt:valueQuantity.
			  ?degree1 owl:minQualifiedCardinality ?res.
                    }
                    """ % (
        foodobject
    )
    full_query = prefix + query
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return (
        results["results"]["bindings"][0]["res"]["value"]
        if results["results"]["bindings"]
        else "0"
    )


def get_max_angle(foodobject):
    query = """
    SELECT ?res WHERE {
                        %s rdfs:subClassOf ?cons_node.
                        ?cons_node owl:onProperty pour:hasConsistency.
			?cons_node owl:someValuesFrom ?consistency.
                        ?union rdf:first*/rdf:rest* ?consistency.
 			 {
 			 ?rest rdf:first*/rdf:rest* ?union.
  			?start owl:unionOf ?rest.
 			 ?a owl:someValuesFrom ?start.
 			 }
 			 UNION
 			 {
 			   ?a owl:allValuesFrom ?union.
 			 }
 			 ?f rdf:first*/rdf:rest* ?a.
			  ?r rdf:first*/rdf:rest* ?f.
			  ?i owl:intersectionOf ?r.
			  ?inter owl:someValuesFrom ?i.
			  ?param rdfs:subClassOf ?inter.
			  ?param rdfs:subClassOf pour:PouringAngle.
			  ?param rdfs:subClassOf ?degree1.
			  ?degree1 owl:onProperty qudt:valueQuantity.
			  ?degree1 owl:maxQualifiedCardinality ?res.
                    }""" % (
        foodobject
    )
    full_query = prefix + query
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return (
        results["results"]["bindings"][0]["res"]["value"]
        if results["results"]["bindings"]
        else "90"
    )


def get_min_duration(foodobject):
    query = """
    SELECT ?res WHERE {
                        %s rdfs:subClassOf ?cons_node.
                        ?cons_node owl:onProperty pour:hasConsistency.
			?cons_node owl:someValuesFrom ?consistency.
                        ?union rdf:first*/rdf:rest* ?consistency.
 			 {
 			 ?rest rdf:first*/rdf:rest* ?union.
  			?start owl:unionOf ?rest.
 			 ?a owl:someValuesFrom ?start.
 			 }
 			 UNION
 			 {
 			   ?a owl:allValuesFrom ?union.
 			 }
 			 ?f rdf:first*/rdf:rest* ?a.
			  ?r rdf:first*/rdf:rest* ?f.
			  ?i owl:intersectionOf ?r.
			  ?inter owl:someValuesFrom ?i.
			  ?param rdfs:subClassOf ?inter.
			  ?param rdfs:subClassOf pour:PouringDuration.
			  ?param rdfs:subClassOf ?degree1.
			  ?degree1 owl:onProperty qudt:valueQuantity.
			  ?degree1 owl:minQualifiedCardinality ?res.
                    }
                    """ % (
        foodobject
    )
    full_query = prefix + query
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return (
        results["results"]["bindings"][0]["res"]["value"]
        if results["results"]["bindings"]
        else "0"
    )


def get_max_duration(foodobject):
    query = """
    SELECT ?res WHERE {
                        %s rdfs:subClassOf ?cons_node.
                        ?cons_node owl:onProperty pour:hasConsistency.
			?cons_node owl:someValuesFrom ?consistency.
                        ?union rdf:first*/rdf:rest* ?consistency.
 			 {
 			 ?rest rdf:first*/rdf:rest* ?union.
  			?start owl:unionOf ?rest.
 			 ?a owl:someValuesFrom ?start.
 			 }
 			 UNION
 			 {
 			   ?a owl:allValuesFrom ?union.
 			 }
 			 ?f rdf:first*/rdf:rest* ?a.
			  ?r rdf:first*/rdf:rest* ?f.
			  ?i owl:intersectionOf ?r.
			  ?inter owl:someValuesFrom ?i.
			  ?param rdfs:subClassOf ?inter.
			  ?param rdfs:subClassOf pour:PouringDuration.
			  ?param rdfs:subClassOf ?degree1.
			  ?degree1 owl:onProperty qudt:valueQuantity.
			  ?degree1 owl:maxQualifiedCardinality ?res.
                    }
    """ % (
        foodobject
    )
    full_query = prefix + query
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return (
        results["results"]["bindings"][0]["res"]["value"]
        if results["results"]["bindings"]
        else "10"
    )


def query_var(verb, foodobject):
    print(
        f"For the verb {verb} and food object {foodobject}, the needed tool is:{get_needed_tool(verb)}"
    )
    print(
        f"For the verb {verb} and food object {foodobject}, the minimum angle is:{get_min_angle(foodobject)}"
    )
    print(
        f"For the verb {verb} and food object {foodobject}, the maximum angle is:{get_max_angle(foodobject)}"
    )
    print(
        f"For the verb {verb} and food object {foodobject}, the minimum duration is:{get_min_duration(foodobject)}"
    )
    print(
        f"For the verb {verb} and food object {foodobject}, the maximum duration is:{get_max_duration(foodobject)}"
    )


verb = "pour:Draining"
foodobject = "obo:FOODON_03301304"

query_var(verb, foodobject)
