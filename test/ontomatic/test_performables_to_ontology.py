from ontomatic.ontomatic_performable_test_dummy import TestOntomaticPerformable
from sqlalchemy.testing import skip_test

from pycram.ontomatic.performables_to_ontology import create_ontology_from_performables
from tempfile import NamedTemporaryFile
import owlready2
import unittest
import os

@unittest.skip
class TestOntologyCreation(unittest.TestCase):
    def test_Performables_parsed(self):
        tmp_file = NamedTemporaryFile(mode="w+", suffix=".owl")
        create_ontology_from_performables(output_path=tmp_file.name)
        ontology = owlready2.get_ontology(tmp_file.name).load()
        performable_class = next(filter(lambda c: c._name == 'Performable', ontology.classes()))
        self.assertTrue(len(performable_class.instances()) > 0)

    def test_Parameters_parsed(self):
        tmp_file = NamedTemporaryFile(mode="w+", suffix=".owl")
        create_ontology_from_performables(output_path=tmp_file.name)
        ontology = owlready2.get_ontology(tmp_file.name).load()
        parameter_class = next(filter(lambda c: c._name == 'Parameter', ontology.classes()))
        self.assertTrue(len(parameter_class.instances()) > 0)

    def test_Enums_parsed(self):
        tmp_file = NamedTemporaryFile(mode="w+", suffix=".owl")
        create_ontology_from_performables(output_path=tmp_file.name)
        ontology = owlready2.get_ontology(tmp_file.name).load()
        enum_class = next(filter(lambda c: c._name == 'Enum', ontology.ontology.classes()))
        self.assertTrue(len(enum_class.instances()) > 0)

    @unittest.skip("Testcase fails with current implementation")
    def test_information_correctly_parsed(self):
        expected_Enum_subclasses = ["GripperState_Value", "Arms_Value"]
        expected_classes = ['Parameter', 'Enum', 'Arms', 'Arms_Value', 'GripperState', 'GripperState_Value', 'bool', 'Performable']
        expected_arm_value_instances = ['BOTH', 'LEFT', 'RIGHT']
        expected_gripper_state_value_instances = ['CLOSE', 'OPEN']
        expected_class_gripper_state_description = "Enum for the different motions of the gripper."
        expected_class_arm_description = "Enum for Arms."
        expected_class_performable_description = "Set the gripper state of the robot."
        expected_Arms_param_name = "gripper"
        expected_Arms_param_description = "The gripper that should be set"
        expected_GripperState_param_name = "motion"
        expected_GripperState_param_description = "The motion that should be set on the gripper"
        expected_bool_param_name = "the_truth"
        expected_bool_param_description = "The true meaning"

        tmp_file = NamedTemporaryFile(mode="w+", suffix=".owl")
        create_ontology_from_performables(output_path=tmp_file.name, abstract_actions_to_parse=TestOntomaticPerformable)
        ontology = owlready2.get_ontology(tmp_file.name).load()

        clazz_GripperState = next(filter(lambda c: c._name == 'GripperState', ontology.classes()))
        clazz_Arms = next(filter(lambda c: c._name == 'Arms', ontology.classes()))
        clazz_Performable = next(filter(lambda c: c._name == 'Performable', ontology.classes()))
        clazz_bool = next(filter(lambda c: c._name == 'bool', ontology.classes()))
        for subclass in next(filter(lambda c: c._name == 'Enum', ontology.classes())).subclasses():
            self.assertIn(subclass._name, expected_Enum_subclasses)
        for clazz in ontology.classes():
            self.assertIn(clazz._name, expected_classes)
        for instance in next(filter(lambda c: c._name == 'Arms_Value', ontology.classes())).instances():
            self.assertIn(instance.name, expected_arm_value_instances)
        for instance in next(filter(lambda c: c._name == 'GripperState_Value', ontology.classes())).instances():
            self.assertIn(instance.name, expected_gripper_state_value_instances)
        self.assertIn(expected_class_gripper_state_description, str(clazz_GripperState.has_description))
        self.assertIn(expected_class_arm_description, str(clazz_Arms.has_description))
        self.assertEqual(clazz_Performable.instances()[0].name, "TestOntomaticPerformable")
        self.assertIn(expected_class_performable_description, str(clazz_Performable.instances()[0].has_description))
        self.assertEqual(expected_Arms_param_name, clazz_Arms.instances()[0].name)
        self.assertIn(expected_Arms_param_description, str(clazz_Arms.instances()[0].has_description))
        self.assertEqual(expected_GripperState_param_name, clazz_GripperState.instances()[0].name)
        self.assertIn(expected_GripperState_param_description, str(clazz_GripperState.instances()[0].has_description))
        self.assertEqual(expected_bool_param_name, clazz_bool.instances()[0].name)
        self.assertIn(expected_bool_param_description, str(clazz_bool.instances()[0].has_description))

if __name__ == '__main__':
    unittest.main()