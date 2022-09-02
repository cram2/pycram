from typing import List
import rosservice
import rospy
import logging

#logging.setLoggerClass(logging.Logger)
logger = logging.getLogger(__name__)
from pycram import ch

logger.addHandler(ch)
logger.setLevel(logging.DEBUG)

if 'rosprolog/query' in rosservice.get_service_list():
    from neem_interface_python.rosprolog_client import atom
    from pycram.knowrob import knowrob
else:
    logger.warning("No KnowRob services found, knowrob is not available")


def object_type(object_iri: str) -> str:
    """
    :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base
    """
    res = knowrob.once(f"kb_call(instance_of({atom(object_iri)}, Class))")
    return res["Class"]


def instances_of(type_: str) -> List[str]:
    """
    :param type_: An object type (i.e. class)
    """
    all_sols = knowrob.all_solutions(f"kb_call(instance_of(Individual, {atom(type_)}))")
    return [sol["Individual"] for sol in all_sols]


def object_pose(object_iri: str, reference_cs: str = "world", timestamp=None) -> List[float]:
    """
    :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base
    :param reference_cs: The coordinate system relative to which the pose should be defined
    """
    if timestamp is None:
        res = knowrob.once(f"mem_tf_get({atom(object_iri)}, {atom(reference_cs)}, Pos, Ori)")
    else:
        res = knowrob.once(f"mem_tf_get({atom(object_iri)}, {atom(reference_cs)}, Pos, Ori, {timestamp})")
    pos = res["Pos"]
    ori = res["Ori"]
    return pos + ori


def grasp_pose(object_iri: str) -> List[float]:
    query = f"""
    kb_call(has_grasp_point({atom(object_iri)}, GraspPointName)),
    mem_tf_get(GraspPointName, world, Pos, Ori)
    """
    res = knowrob.once(query)
    pos = res["Pos"]
    ori = res["Ori"]
    return pos + ori
