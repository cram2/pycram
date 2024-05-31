from __future__ import annotations

import unittest
import logging
from pathlib import Path
from typing import Type, Optional

from pycram.designator import ObjectDesignatorDescription

import rospy
try:
    from owlready2 import *
except ImportError:
    owlready2 = None
    rospy.logwarn("Could not import owlready2, Ontology unit-tests could not run!")

from pycram.ontology.ontology import OntologyManager, SOMA_ONTOLOGY_IRI
from pycram.ontology.ontology_common import OntologyConceptHolderStore, OntologyConceptHolder


class TestOntologyManager(unittest.TestCase):
    ontology_manager: OntologyManager
    main_ontology: Optional[owlready2.Ontology]
    soma: Optional[owlready2.Ontology]
    dul: Optional[owlready2.Ontology]

    @classmethod
    def setUpClass(cls):
        cls.ontology_manager = OntologyManager(SOMA_ONTOLOGY_IRI)
        if cls.ontology_manager.initialized():
            cls.main_ontology = cls.ontology_manager.main_ontology
            cls.soma = cls.ontology_manager.soma
            cls.dul = cls.ontology_manager.dul
        else:
            cls.main_ontology = None
            cls.soma = None
            cls.dul = None

    @classmethod
    def tearDownClass(cls):
        if cls.ontology_manager.initialized():
            save_dir = Path(f"{Path.home()}/ontologies")
            save_dir.mkdir(parents=True, exist_ok=True)
            cls.ontology_manager.save(f"{save_dir}/{Path(cls.ontology_manager.main_ontology_iri).stem}.owl")

    def test_ontology_manager(self):
        if self.ontology_manager.initialized():
            self.assertIs(self.ontology_manager, OntologyManager())

    def test_ontology_concept_holder(self):
        if not self.ontology_manager.initialized():
            return
        dynamic_ontology_concept_class = self.ontology_manager.create_ontology_concept_class('DynamicOntologyConcept')
        dynamic_ontology_concept_holder = OntologyConceptHolder(dynamic_ontology_concept_class(name='dynamic_ontology_concept1',
                                                                                               namespace=self.main_ontology))
        self.assertTrue(owlready2.isinstance_python(dynamic_ontology_concept_holder.ontology_concept, owlready2.Thing))

    def test_loaded_ontologies(self):
        if not self.ontology_manager.initialized():
            return
        self.assertIsNotNone(self.main_ontology)
        self.assertTrue(self.main_ontology.loaded)
        self.assertIsNotNone(self.soma)
        self.assertTrue(self.soma.loaded)
        self.assertIsNotNone(self.dul)
        self.assertTrue(self.dul.loaded)

    def test_ontology_concept_class_dynamic_creation(self):
        if not self.ontology_manager.initialized():
            return
        dynamic_ontology_concept_class = self.ontology_manager.create_ontology_concept_class('DynamicOntologyConcept')
        self.assertIsNotNone(dynamic_ontology_concept_class)
        self.assertEqual(dynamic_ontology_concept_class.namespace, self.main_ontology)
        self.assertIs(dynamic_ontology_concept_class, self.main_ontology.DynamicOntologyConcept)
        self.assertIs(issubclass(dynamic_ontology_concept_class, owlready2.Thing), True)
        dynamic_ontology_concept = dynamic_ontology_concept_class(name='dynamic_ontology_concept2',
                                                                  namespace=self.main_ontology)
        self.assertTrue(owlready2.isinstance_python(dynamic_ontology_concept, owlready2.Thing))

    def test_ontology_triple_classes_dynamic_creation(self):
        if not self.ontology_manager.initialized():
            return
        # Test dynamic triple classes creation without inheritance from existing parent ontology classes
        self.ontology_manager.create_ontology_triple_classes(subject_class_name="OntologySubject",
                                                             object_class_name="OntologyObject",
                                                             predicate_name="predicate",
                                                             inverse_predicate_name="inverse_predicate")

        subject_class = self.main_ontology.OntologySubject
        self.assertIsNotNone(subject_class)
        subject_individual = subject_class("subject")
        self.assertIsNotNone(subject_individual.predicate)

        object_class = self.main_ontology.OntologyObject
        self.assertIsNotNone(object_class)
        object_individual = object_class("object")
        self.assertIsNotNone(object_individual.inverse_predicate)

        # Test dynamic triple classes creation as inheriting from existing parent ontology classes
        PLACEABLE_ON_PREDICATE_NAME = "placeable_on"
        HOLD_OBJ_PREDICATE_NAME = "hold_obj"
        self.ontology_manager.create_ontology_triple_classes(ontology_subject_parent_class=self.soma.Container,
                                                             subject_class_name="OntologyPlaceHolderObject",
                                                             ontology_object_parent_class=self.dul.PhysicalObject,
                                                             object_class_name="OntologyHandheldObject",
                                                             predicate_name=PLACEABLE_ON_PREDICATE_NAME,
                                                             inverse_predicate_name=HOLD_OBJ_PREDICATE_NAME,
                                                             ontology_property_parent_class=self.soma.affordsBearer,
                                                             ontology_inverse_property_parent_class=self.soma.isBearerAffordedBy)

        def create_ontology_handheld_object_designator(object_name: str, ontology_parent_class: Type[owlready2.Thing]):
            return self.ontology_manager.create_ontology_linked_designator(object_name=object_name,
                                                                           designator_class=ObjectDesignatorDescription,
                                                                           ontology_concept_name=f"Onto{object_name}",
                                                                           ontology_parent_class=ontology_parent_class)

        # Holdable object
        egg = create_ontology_handheld_object_designator("egg", self.main_ontology.OntologyHandheldObject)
        # Placeholder object
        egg_tray = create_ontology_handheld_object_designator("egg_tray", self.main_ontology.OntologyPlaceHolderObject)

        # Create ontology relation between [Place-holder] and [Holdable obj]
        self.ontology_manager.set_ontology_relation(subject_designator=egg, object_designator=egg_tray,
                                                    predicate_name=PLACEABLE_ON_PREDICATE_NAME)

        self.ontology_manager.set_ontology_relation(subject_designator=egg_tray, object_designator=egg,
                                                    predicate_name=HOLD_OBJ_PREDICATE_NAME)

        # Query potential designator candidates based on above-set ontology relations among them
        egg_placeholders = [placeholder.names for placeholder in \
                            self.ontology_manager.get_designators_by_subject_predicate(subject=egg,
                                                                                       predicate_name=PLACEABLE_ON_PREDICATE_NAME)]
        self.assertTrue(len(egg_placeholders) == 1)
        self.assertEqual(egg_placeholders[0], ["egg_tray"])

        egg_tray_holdables = [placeholder.names for placeholder in \
                              self.ontology_manager.get_designators_by_subject_predicate(subject=egg_tray,
                                                                                         predicate_name=HOLD_OBJ_PREDICATE_NAME)]
        self.assertTrue(len(egg_tray_holdables) == 1)
        self.assertEqual(egg_tray_holdables[0], ["egg"])

    def test_ontology_class_destruction(self):
        if not self.ontology_manager.initialized():
            return
        concept_class_name = 'DynamicOntologyConcept'
        dynamic_ontology_concept_class = self.ontology_manager.create_ontology_concept_class(concept_class_name)
        OntologyConceptHolder(dynamic_ontology_concept_class(name='dynamic_ontology_concept3',
                                                             namespace=self.main_ontology))

        self.ontology_manager.destroy_ontology_class(dynamic_ontology_concept_class)
        self.assertIsNone(self.ontology_manager.get_ontology_class(concept_class_name))
        self.assertFalse(OntologyConceptHolderStore().get_ontology_concepts_by_class(dynamic_ontology_concept_class))


if __name__ == '__main__':
    unittest.main()
