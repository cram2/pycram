import logging
import os

from typing import Dict, List, Union

import sys
SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, os.pardir, os.pardir, "neem-interface", "src"))

from neem_interface_python.neem_interface import NEEMInterface
from neem_interface_python.rosprolog_client import Prolog, PrologException

neem_interface = NEEMInterface()
prolog = Prolog()


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
    Example: load_owl("package://knowrob/owl/maps/iai_room_v1.owl", "map", "http://knowrob.org/kb/v1/IAI-Kitchen.owl#")
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
