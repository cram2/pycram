import re
import xml.etree.ElementTree as ET
import os
from typing import List, Any

import pycrap
from pycrap.ontologies import FixedJoint, PlanarJoint, RevoluteJoint, FloatingJoint, HingeJoint, ContinuousJoint, \
    PrismaticJoint

def remove_digits(string):
    return re.sub(r'\d+', '', string)

def parse_furniture(link):
    #furniture = ["Door", "Fridge", "Handle", "Sink", "Drawer", "Table", "Leg"]
    matched_furniture = []

    for l in link.split('_'):
        for c in pycrap.ontologies.base.ontology.classes():
            clean_l = remove_digits(l).capitalize()  # Remove digits and capitalize
            # print(f"clsname: {c[0]}, link: {clean_l}")
            if clean_l == str(c).split(".")[1]:
                matched_furniture.append(c)
    if matched_furniture:
        return matched_furniture[len(matched_furniture)-1]

def parse_joint_types(joint):
    joint_types = [FixedJoint, PlanarJoint, RevoluteJoint, FloatingJoint, HingeJoint, ContinuousJoint, PrismaticJoint]
    for j in joint_types:
        if str(joint).lower() in str(j).lower():
            return j

class URDFParser:


    base_file_name: str = "base"

    classes_file_name: str = "classes"

    properties_file_name: str = "properties"

    restrictions_file_name: str = "restrictions"

    individuals_file_name: str = "individuals"

    file_extension: str = ".py"

    indentation = 4

    def __init__(self, urdf_file: str, output_dir: str):
        """
        Initialize the URDFParser with the URDF file and output directory.
        """
        self.urdf_file = urdf_file
        self.output_dir = output_dir
        self.links: List[str] = []
        self.joints: List[dict] = []
        self._parse_urdf()

    def _parse_urdf(self):
        """
        Parse the URDF file to extract links and joints.
        """

        tree = ET.parse(self.urdf_file)
        root = tree.getroot()

        for child in root:
            if child.tag == 'link':
                 self.links.append(child.attrib['name'])
            elif child.tag == 'joint':
                joint_info = {
                        'name': child.attrib['name'],
                        'type': child.attrib.get('type'),
                        'parent': child.find('parent').attrib['link'],
                        'child': child.find('child').attrib['link']
                }
                self.joints.append(joint_info)


    def parse_furniture(self, link):
        """
        Parse the furniture file.
        Test it on a hardcoded furniture list, later use an actual ontology file for it.

        """
        # furniture = imported_ontology.classes()
        furniture = ["Door", "Fridge", "Handle", "Sink", "Drawer"]
        matching_list = []
        for l in link.split("_"):
            if l.capitalize() in furniture:
                matched_furniture = furniture.index(l.capitalize())
                print(f"Here it is, Found: {l} from Link: {link}")
                print(f"Restriction would be:{l}.is_a = [{furniture[matched_furniture]}] ")
                matching_list.append(furniture[matched_furniture])


        return matching_list


    def apply_indent_to(self, string: str) -> str:
        """
        Indent a statement at the beginning of every new line.
        """
        return " " * self.indentation + string.replace('\n', '\n' + ' ' * self.indentation)

    def path_for_file(self, file_name: str) -> str:
        """
        Generate the full path for a file in the output directory.
        """
        return os.path.join(self.output_dir, f"{file_name}{self.file_extension}")

    def generate_base_imports(self):
        """
        Generate a Python file containing the base class.
        """
        base_file_path = self.path_for_file(self.base_file_name)
        with open(base_file_path, "w") as file:
            # Write imports and setup
            file.write("from owlready2 import Thing, ObjectProperty\n")
            file.write("import tempfile\n\n")
            file.write("import owlready2\n\n")
            file.write("ontology_file = tempfile.NamedTemporaryFile()\n")
            file.write('ontology = owlready2.get_ontology("file://" + ontology_file.name).load()\n\n')

            # Write the Base class
            file.write("class Base(Thing):\n")
            file.write("    \"\"\"Base class for all links and joints.\"\"\"\n")
            file.write("    namespace = ontology\n\n")

            # Write the BaseProperty class
            file.write("class BaseProperty(ObjectProperty):\n")
            file.write("    \"\"\"Base property for object properties.\"\"\"\n")
            file.write("    namespace = ontology\n\n")

        print(f"Base class written to {base_file_path}")


    def generate_classes_file(self):
        """
        Generate a Python file with classes for all links and joints.

        Classes Joints, Links and the Joint types are always defined.

        Add some Furniture classes, later import another ontology.

        """
        classes_file_path = self.path_for_file(self.classes_file_name)
        with open(classes_file_path, "w") as file:
            file.write("from base import *\n\n\n")
            file.write("class Links(Base):\n")
            file.write("    pass\n\n")

            file.write("class Joints(Base):\n")
            file.write("    pass\n\n")

            file.write("class Fixed(Base):\n")
            #file.write("    \"\"\"Fixed base class for all links and joints.\"\"\"\n")
            file.write("    pass\n\n")

            file.write("class Revolute(Base):\n")
            # file.write("    \"\"\"Fixed base class for all links and joints.\"\"\"\n")
            file.write("    pass\n\n")

            file.write("class Prismatic(Base):\n")
            # file.write("    \"\"\"Fixed base class for all links and joints.\"\"\"\n")
            file.write("    pass\n\n")

            file.write("class Planar(Base):\n")
            # file.write("    \"\"\"Fixed base class for all links and joints.\"\"\"\n")
            file.write("    pass\n\n")

            file.write("class Continous(Base):\n")
            # file.write("    \"\"\"Fixed base class for all links and joints.\"\"\"\n")
            file.write("    pass\n\n")

            file.write("class Door(Base):\n")
            # file.write("    \"\"\"Fixed base class for all links and joints.\"\"\"\n")
            file.write("    pass\n\n")

            file.write("class Fridge(Base):\n")
            # file.write("    \"\"\"Fixed base class for all links and joints.\"\"\"\n")
            file.write("    pass\n\n")

            file.write("class Handle(Base):\n")
            # file.write("    \"\"\"Fixed base class for all links and joints.\"\"\"\n")
            file.write("    pass\n\n")

            file.write("class Sink(Base):\n")
            # file.write("    \"\"\"Fixed base class for all links and joints.\"\"\"\n")
            file.write("    pass\n\n")

            file.write("class Drawer(Base):\n")
            # file.write("    \"\"\"Fixed base class for all links and joints.\"\"\"\n")
            file.write("    pass\n\n")

        print(f"Classes written to {classes_file_path}")


    def generate_individuals_file(self):
        """
        Generate a Python file with individuals.
        """
        individuals_file_path = self.path_for_file(self.individuals_file_name)
        with open(individuals_file_path, "w") as file:
            file.write("from properties import *\n\n\n")
            for link in self.links:
                file.write(f"{link} = Links('{link}')\n")

            for joint in self.joints:
                file.write(f"{joint['name']} = Joints('{joint['name']}')\n")

    def generate_properties_file(self):
        """
        Generate a Python file with properties for joints.
        """
        properties_file_path = self.path_for_file(self.properties_file_name)
        with open(properties_file_path, "w") as file:
            # Write imports at the top
            file.write("from classes import *\n\n")

            # Write the HasParentLink class
            file.write("class hasParentLink(BaseProperty):\n")
            file.write("    \"\"\"Property to link a joint to its parent link.\"\"\"\n")
            file.write("    pass\n\n")  # Properly indented pass statement

            # Write the HasChildLink class
            file.write("class hasChildLink(BaseProperty):\n")
            file.write("    \"\"\"Property to link a joint to its child link.\"\"\"\n")
            file.write("    pass\n\n")  # Properly indented pass statement

            # Define the IsPartOf class
            file.write("class isPartOf(BaseProperty):\n")
            file.write("    \"\"\"Property to establish part-of relationships between links.\"\"\"\n")
            file.write("    pass\n\n")

        print(f"Properties written to {properties_file_path}")

    def generate_restrictions_file(self):
        """
        Generate a Python file with restrictions for joints, including joint types and relationships.
        """
        restrictions_file_path = self.path_for_file(self.restrictions_file_name)

        with open(restrictions_file_path, "w") as file:
            # Write the header
            file.write("from individuals import *\n\n")
            # Generate the restrictions for joints
            file.write("Fixed.is_a = [Joints]\n\n")
            file.write("Revolute.is_a = [Joints]\n\n")
            file.write("Prismatic.is_a = [Joints]\n\n")
            file.write("Planar.is_a = [Joints]\n\n")
            file.write("Continous.is_a = [Joints]\n\n")

            for joint in self.joints:
                instance_name = joint['name']
                joint_type = joint['type'].capitalize()
                parent_link = joint['parent']
                child_link = joint['child']

                file.write(f"{instance_name}.is_a = [{joint_type}]\n")
                file.write(f"{instance_name}.hasChildLink.append({joint['child']})\n")
                file.write(f"{instance_name}.hasParentLink = [{joint['parent']}]\n")
                file.write(f"{child_link}.isPartOf = [{parent_link}]\n")
                file.write("\n")

            for link in self.links:
                matched_furnitures = self.parse_furniture(link)
                if matched_furnitures:
                    #file.write(f"{link}.is_a = [Links, {', '.join(matched_furnitures)}]\n")
                    file.write(f"{link}.is_a = [Links, {matched_furnitures[len(matched_furnitures) - 1]}]\n")

        print(f"Restrictions written to {restrictions_file_path}")

    def generate_all(self):
        """
        Generate all required Python files.
        """
        self.generate_base_imports()
        self.generate_classes_file()
        self.generate_properties_file()
        self.generate_restrictions_file()
        self.generate_individuals_file()