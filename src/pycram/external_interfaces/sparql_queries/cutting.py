import owlready2
from SPARQLWrapper import SPARQLWrapper, JSON

sparql = SPARQLWrapper("https://knowledgedb.informatik.uni-bremen.de/mealprepDB/MealPreparation/query")
sparql.setReturnFormat(JSON)

prefix = """
 PREFIX owl: <http://www.w3.org/2002/07/owl#>
 PREFIX cut: <http://www.ease-crc.org/ont/meals#>
 PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
 PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
 PREFIX foodon: <http://purl.obolibrary.org/obo/>
 PREFIX soma: <http://www.ease-crc.org/ont/SOMA.owl#>
 PREFIX sit_aware: <http://www.ease-crc.org/ont/situation_awareness#>
 """

def check_food_part(food, part):
    query = """
    ASK {
					  foodon:%s rdfs:subClassOf* ?dis.
					  ?dis owl:onProperty cut:hasPart.
					  ?dis owl:someValuesFrom ?tar.
					  ?tar owl:intersectionOf ?tar_int.
					  ?tar_int rdf:first cut:%s.
					  ?tar_int rdf:rest ?rest.
					  ?rest rdf:first ?first.
					  ?first owl:onProperty cut:hasEdibility.
					  {
						?first owl:someValuesFrom cut:MustBeAvoided.
					  }
					  UNION
					  {
						?first owl:someValuesFrom cut:ShouldBeAvoided.
					  }
					}
    """% (food, part)
    full_query = (prefix + query)
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return results["boolean"]

def get_prior_task(verb):
    query = """
    SELECT ?res WHERE {
						%s rdfs:subClassOf* ?sub.
						?sub owl:onProperty cut:requiresPriorTask .
						?sub owl:someValuesFrom ?priortask.
						BIND(REPLACE(STR(?priortask), "^.*[#/]", "") AS ?res).
					}
					""" % (verb)
    full_query = (prefix + query)
    sparql.setQuery(prefix + query)
    results = sparql.queryAndConvert()
    return results["results"]["bindings"][0]["res"]["value"] if results["results"]["bindings"] else None

def get_cutting_tool(foodobject):
    query = """
    SELECT ?res WHERE {
							foodon:%s rdfs:subClassOf* ?peel_dis.
							?peel_dis owl:onProperty soma:hasDisposition.
							?peel_dis owl:someValuesFrom ?peel_dis_vals.
							?peel_dis_vals owl:intersectionOf ?afford_vals.
							?afford_vals rdf:first sit_aware:Cuttability.
							?afford_vals rdf:rest ?task_trigger.
							?task_trigger rdf:rest ?trigger.
							?trigger rdf:first ?trigger_wo_nil.
							?trigger_wo_nil owl:onProperty soma:affordsTrigger.
							?trigger_wo_nil owl:allValuesFrom ?trigger_tool.
							?trigger_tool owl:allValuesFrom ?tool.
							BIND(REPLACE(STR(?tool), "^.*[#/]", "") AS ?res).
						}
    """ % (foodobject)
    full_query = (prefix + query)
    sparql.setQuery(prefix + query)
    results = sparql.queryAndConvert()
    return results["results"]["bindings"][0]["res"]["value"] if results["results"]["bindings"] else "Knife"

def get_cutting_position(verb):
    query = """
    SELECT * WHERE {
						%s rdfs:subClassOf* ?pos_node.
						?pos_node owl:onProperty cut:affordsPosition.
						?pos_node owl:someValuesFrom ?pos.
						BIND(REPLACE(STR(?pos), "^.*[#/]", "") AS ?res).
					}
    """ % (verb)
    full_query = (prefix + query)
    sparql.setQuery(prefix + query)
    results = sparql.queryAndConvert()
    return results["results"]["bindings"][0]["res"]["value"] if results["results"]["bindings"] else "middle"

def get_repetition(verb):
    query = """
    SELECT ?res WHERE {
					  {
						%s rdfs:subClassOf* ?rep_node.
						?rep_node owl:onProperty cut:repetitions.
						FILTER EXISTS {
							?rep_node owl:hasValue ?val.
						}
						BIND("exactly 1" AS ?res)
					  }
					  UNION
					  {
						%s rdfs:subClassOf* ?rep_node.
						?rep_node owl:onProperty cut:repetitions.
						FILTER EXISTS {
							?rep_node owl:minQualifiedCardinality ?val.
						}
						BIND("at least 1" AS ?res)
					  }
					}
    """ % (verb, verb)
    full_query = (prefix + query)
    sparql.setQuery(prefix + query)
    results = sparql.queryAndConvert()
    return results["results"]["bindings"][0]["res"]["value"] if results["results"]["bindings"] else "1"

def get_peel_tool(foodobject):
    query = """
    SELECT ?res WHERE {
						foodon:%s rdfs:subClassOf* ?peel_dis.
						?peel_dis owl:onProperty soma:hasDisposition.
						?peel_dis owl:someValuesFrom ?peel_dis_vals.
						?peel_dis_vals owl:intersectionOf ?afford_vals.
						?afford_vals rdf:first cut:Peelability.
						?afford_vals rdf:rest ?task_trigger.
						?task_trigger rdf:rest ?trigger.
						?trigger rdf:first ?trigger_wo_nil.
						?trigger_wo_nil owl:onProperty soma:affordsTrigger.
						?trigger_wo_nil owl:allValuesFrom ?trigger_tool.
						?trigger_tool owl:allValuesFrom ?tool.
						BIND(REPLACE(STR(?tool), "^.*[#/]", "") AS ?res).
					}
    """ % (foodobject)
    full_query = (prefix + query)
    sparql.setQuery(prefix + query)
    results = sparql.queryAndConvert()
    return results["results"]["bindings"][0]["res"]["value"] if results["results"]["bindings"] else "Peeler"

def query_var(verb, foodobject):
    priorTask = get_prior_task(verb)
    cut_tool = get_cutting_tool(foodobject)
    cut_position = get_cutting_position(verb)
    repetition = get_repetition(verb)
    remove_peel = check_food_part(foodobject, "Peel")
    remove_core = check_food_part(foodobject, "Core")
    remove_stem = check_food_part(foodobject, "Stem")
    remove_shell = check_food_part(foodobject, "Shell")


    print(f"For {verb} on {foodobject}, the prior task is: {priorTask}, and the cutting tool is: {cut_tool}, and the cutting position is: {cut_position},"
          f" and the repetition is: {repetition}")

    print(f"Remove peel: {remove_peel}, Remove core: {remove_core}, Remove stem: {remove_stem}, Remove shell: {remove_shell}")

    if remove_peel or remove_shell:
        peeling_tool = get_peel_tool(foodobject)
        print(f"Peeling tool: {peeling_tool}")
    """
    Bei Stem und Core wird einfach "Remove Core/Stem" ausgegeben, aber es gibt keine Abfrage für den Tool
    """

verb = "cut:Quartering"
foodobject = "FOODON_00003523"
query_var(verb, foodobject)