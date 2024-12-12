import os
from typing import List

import pycrap


def parse_furniture(link):
    #furniture = ["Door", "Fridge", "Handle", "Sink", "Drawer", "Table", "Leg"]
    matched_furniture = []
    for c in pycrap.ontologies.base.ontology.classes():
        for l in link.split('_'):
            if l.capitalize() in str(c):
                matched_furniture.append(c)

    return matched_furniture[len(matched_furniture)-1]

class URDFParser:


    individuals_file_name: str = "individuals"

    restrictions_file_name: str = "restrictions"

    file_extension: str = ".py"


    def __init__(self, output_dir: str, object_description):
    #def __init__(self, output_dir: str, links: List[str], joints: List[dict], object_description):

        self.output_dir = output_dir
        #self.links = links
        #self.joints = joints
        self.object_description = object_description


    def parse_description(self):
        links = []
        joints = []
        for j in self.object_description.joints:
            joint = self.object_description.joints[j]
            joint_info = {
                'name': j,
                'type': joint.type.name,
                'parent': joint.parent,
                'child': joint.child
            }
            joints.append(joint_info)
        for i in self.object_description.links:
            links.append(i)

        return links, joints

    def path_for_file(self, file_name: str) -> str:
        """
        Generate the full path for a file in the output directory.
        """
        return os.path.join(self.output_dir, f"{file_name}{self.file_extension}")

    def generate_individuals_file(self):
        """
        Generate a Python file with individuals.
        """
        individuals_file_path = self.path_for_file(self.individuals_file_name)
        with open(individuals_file_path, "w") as file:
            for link in self.parse_description()[0]:
                file.write(f"{link} = Links('{link}')\n")

            file.write(f"\n")

            for joint in self.parse_description()[1]:
                file.write(f"{joint['name']} = Joints('{joint['name']}')\n")

            file.write(f"\n")

            furniture = ["Door", "Fridge", "Handle", "Sink", "Drawer", "Table", "Leg"]

            #matching_list = []

            for link in self.parse_description()[0]:
                for l in link.split("_"):
                    if l.capitalize() in furniture:
                        matched_furniture = furniture.index(l.capitalize())
                        file.write(f"{link}.is_a = [{furniture[matched_furniture]}] \n")
                        #matching_list.append(furniture[matched_furniture])

            file.write(f"\n")

            for joint in self.parse_description()[1]:
                instance_name = joint['name']
                joint_type = joint['type']
                parent_link = joint['parent']
                child_link = joint['child']

                file.write(f"{instance_name}.is_a = [{joint_type}]\n")
                file.write(f"{instance_name}.hasChildLink.append({joint['child']})\n")
                file.write(f"{instance_name}.hasParentLink = [{joint['parent']}]\n")
                file.write(f"{child_link}.isPartOf = [{parent_link}]\n")
                file.write("\n")

