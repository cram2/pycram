import logging
import os
import sys
import rosservice

from typing_extensions import Dict, List, Union

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, os.pardir, os.pardir, "neem-interface", "src"))

if 'rosprolog/query' in rosservice.get_service_list():
    from neem_interface_python.neem_interface import NEEMInterface
    from neem_interface_python.rosprolog_client import Prolog, PrologException, atom

    neem_interface = NEEMInterface()
    prolog = Prolog()

else:
    logging.warning("No KnowRob services found, knowrob is not available")


#logging.setLoggerClass(logging.Logger)
logger = logging.getLogger(__name__)
from pycram import ch

logger.addHandler(ch)
logger.setLevel(logging.DEBUG)


def all_solutions(q):
    logging.info(q)
    r = prolog.all_solutions(q)
    return r


def once(q) -> Union[List, Dict]:
    r = all_solutions(q)
    if len(r) == 0:
        return []
    return r[0]


def load_beliefstate(path: str):
    logging.info(f"Restoring beliefstate from {path}")
    once(f"remember('{path}')")


def clear_beliefstate():
    logging.info("Clearing beliefstate")
    once("mem_clear_memory")


def load_owl(path, ns_alias=None, ns_url=None):
    """
    Example: load_owl("package://external_interfaces/owl/maps/iai_room_v1.owl", "map", "http://knowrob.org/kb/v1/IAI-Kitchen.owl#")
    :param str path: path to log folder
    :rtype: bool
    """
    if ns_alias is None or ns_url is None:            # Load without namespace
        q = "load_owl('{}')".format(path)
    else:
        q = "load_owl('{0}', [namespace({1},'{2}')])".format(path, ns_alias, ns_url)
    try:
        once(q)
        return True
    except PrologException as e:
        logging.warning(e)
        return False


def new_iri(owl_class: str):
    res = once(f"kb_call(new_iri(IRI, {owl_class}))")
    return res["IRI"]


def object_type(object_iri: str) -> str:
    """
    :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base
    """
    res = once(f"kb_call(instance_of({atom(object_iri)}, Class))")
    return res["Class"]


def instances_of(type_: str) -> List[str]:
    """
    :param type_: An object type (i.e. class)
    """
    all_sols = all_solutions(f"kb_call(instance_of(Individual, {atom(type_)}))")
    return [sol["Individual"] for sol in all_sols]


def object_pose(object_iri: str, reference_cs: str = "world", timestamp=None) -> List[float]:
    """
    :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base
    :param reference_cs: The coordinate system relative to which the pose should be defined
    """
    if timestamp is None:
        res = once(f"mem_tf_get({atom(object_iri)}, {atom(reference_cs)}, Pos, Ori)")
    else:
        res = once(f"mem_tf_get({atom(object_iri)}, {atom(reference_cs)}, Pos, Ori, {timestamp})")
    pos = res["Pos"]
    ori = res["Ori"]
    return pos + ori


def grasp_pose(object_iri: str) -> List[float]:
    query = f"""
    kb_call(has_grasp_point({atom(object_iri)}, GraspPointName)),
    mem_tf_get(GraspPointName, world, Pos, Ori)
    """
    res = once(query)
    pos = res["Pos"]
    ori = res["Ori"]
    return pos + ori


def knowrob_string_to_pose(pose_as_string: str) -> List[float]:
    reference_frame = ""
    for i, char in enumerate(pose_as_string[1:-1]):
        if char == ",":
            break
        reference_frame += char
    pos, ori = pose_as_string[1+i+2:-2].split("],[")
    xyz = list(map(float, pos.split(",")))
    qxyzw = list(map(float, ori.split(",")))
    return xyz + qxyzw
