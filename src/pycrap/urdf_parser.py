import re
import xml.etree.ElementTree as ET
import os
from typing import List, Any

import pycrap
from pycrap.ontologies import FixedJoint, PlanarJoint, RevoluteJoint, FloatingJoint, HingeJoint, ContinuousJoint, \
    PrismaticJoint

def remove_digits(string):
    return re.sub(r'\d+', '', string)



def parse_furniture(link: str):
    """
    Matching the link names from the parsed description with the given classes in the Ontology.
    This procedure should be replaced with Ontology Tinder.
    """
    #furniture = ["Door", "Fridge", "Handle", "Sink", "Drawer", "Table", "Leg"]
    matched_furniture = []

    for l in link.split('_'):
        for c in pycrap.ontologies.base.ontology.classes():
            if c == pycrap.ontologies.Base:
                continue
            clean_l = remove_digits(l).capitalize()  # Remove digits and capitalize
            # print(f"clsname: {c[0]}, link: {clean_l}")
            splitted = str(c).split(".")

            # TODO: I dont get why this can happen. All ontology classes should be prefixed with some module name.
            if len(splitted) == 1:
                return None
            if clean_l == splitted[1]:
                matched_furniture.append(c)
    if matched_furniture:
        return matched_furniture[len(matched_furniture)-1]

def parse_joint_types(joint):

    """
    Parsing the Joint types coming from the joint description, to be added as type of the corresponding classes
    of PyCRAP.
    """
    joint_types = [FixedJoint, PlanarJoint, RevoluteJoint, FloatingJoint, HingeJoint, ContinuousJoint, PrismaticJoint]
    for j in joint_types:
        if str(joint).lower() in str(j).lower():
            return j