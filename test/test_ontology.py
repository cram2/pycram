from __future__ import annotations

import os.path
import unittest
import logging
from pathlib import Path
from typing import Type, Optional

from pycram.designator import ObjectDesignatorDescription

import rospy

# Owlready2
try:
    from owlready2 import *
except ImportError:
    owlready2 = None
    rospy.logwarn("Could not import owlready2, Ontology unit-tests could not run!")

# Java runtime, required by Owlready2 reasoning
java_runtime_installed = owlready2 is not None
if owlready2:
    try:
        subprocess.run(["java", "--version"], check=True)
    except (FileNotFoundError, subprocess.CalledProcessError):
        java_runtime_installed = False
        rospy.logwarn("Java runtime is not installed, Ontology reasoning unit-test could not run!")

from pycram.ontology.ontology import OntologyManager, SOMA_HOME_ONTOLOGY_IRI, SOMA_ONTOLOGY_IRI
from pycram.ontology.ontology_common import (OntologyConceptHolderStore, OntologyConceptHolder,
                                             ONTOLOGY_SQL_BACKEND_FILE_EXTENSION, ONTOLOGY_OWL_FILE_EXTENSION,
                                             ONTOLOGY_SQL_IN_MEMORY_BACKEND)

DEFAULT_LOCAL_ONTOLOGY_IRI = "default.owl"
class TestOntologyManager(unittest.TestCase):
    ontology_manager: OntologyManager
    main_ontology: Optional[owlready2.Ontology]
    soma: Optional[owlready2.Ontology]
    dul: Optional[owlready2.Ontology]

    @classmethod
    def setUpClass(cls):
        # Try loading from remote `SOMA_ONTOLOGY_IRI`, which will fail given no internet access
        cls.ontology_manager = OntologyManager(main_ontology_iri=SOMA_ONTOLOGY_IRI,
                                               main_sql_backend_filename=os.path.join(Path.home(),
                                               f"{Path(SOMA_ONTOLOGY_IRI).stem}{ONTOLOGY_SQL_BACKEND_FILE_EXTENSION}"))
        if cls.ontology_manager.initialized():
            cls.soma = cls.ontology_manager.soma
            cls.dul = cls.ontology_manager.dul
        else:
            # Else, load from `DEFAULT_LOCAL_ONTOLOGY_IRI`
            cls.soma = None
            cls.dul = None
            cls.ontology_manager.main_ontology_iri = DEFAULT_LOCAL_ONTOLOGY_IRI
            cls.ontology_manager.main_ontology_sql_backend = ONTOLOGY_SQL_IN_MEMORY_BACKEND
            cls.ontology_manager.create_main_ontology_world()
            cls.ontology_manager.create_main_ontology()
        cls.main_ontology = cls.ontology_manager.main_ontology

    @classmethod
    def tearDownClass(cls):
        save_dir = cls.ontology_manager.get_main_ontology_dir()
        owl_filepath = f"{save_dir}/{Path(cls.ontology_manager.main_ontology_iri).stem}{ONTOLOGY_OWL_FILE_EXTENSION}"
        os.remove(owl_filepath)
        cls.remove_sql_file(cls.ontology_manager.main_ontology_sql_backend)

    @classmethod
    def remove_sql_file(cls, sql_filepath: str):
        if os.path.exists(sql_filepath):
            os.remove(sql_filepath)
        sql_journal_filepath = f"{sql_filepath}-journal"
        if os.path.exists(sql_journal_filepath):
            os.remove(sql_journal_filepath)

    def test_ontology_manager(self):
        self.assertIs(self.ontology_manager, OntologyManager())
        if owlready2:
            self.assertTrue(self.ontology_manager.initialized())

    @unittest.skipUnless(owlready2, 'Owlready2 is required')
    def test_ontology_world(self):
        # Main ontology world as the global default world
        main_world = self.ontology_manager.main_ontology_world
        self.assertIsNotNone(main_world)
        self.assertTrue(main_world is owlready2.default_world)

        # Extra world with memory backend
        extra_memory_world = self.ontology_manager.create_ontology_world(use_global_default_world=False)
        self.assertIsNotNone(extra_memory_world)
        self.assertTrue(extra_memory_world != owlready2.default_world)

        # Extra world with SQL backend from a non-existing SQL file
        extra_world_sql_filename = f"{self.ontology_manager.get_main_ontology_dir()}/extra_world{ONTOLOGY_SQL_BACKEND_FILE_EXTENSION}"
        extra_sql_world = self.ontology_manager.create_ontology_world(use_global_default_world=False,
                                                                      sql_backend_filename=extra_world_sql_filename)
        self.assertIsNotNone(extra_sql_world)
        # Save it at [extra_world_sql_filename]
        extra_sql_world.save()
        self.assertTrue(os.path.isfile(extra_world_sql_filename))

        # Extra world with SQL backend from an existing SQL file
        extra_sql_world_2 = self.ontology_manager.create_ontology_world(use_global_default_world=False,
                                                                        sql_backend_filename=extra_world_sql_filename)
        self.assertIsNotNone(extra_sql_world_2)

        # Remove SQL file finally
        self.remove_sql_file(extra_world_sql_filename)

    @unittest.skipUnless(owlready2, 'Owlready2 is required')
    def test_ontology_concept_holder(self):
        dynamic_ontology_concept_class = self.ontology_manager.create_ontology_concept_class('DynamicOntologyConcept')
        dynamic_ontology_concept_holder = OntologyConceptHolder(
            dynamic_ontology_concept_class(name='dynamic_ontology_concept1',
                                           namespace=self.main_ontology))
        self.assertTrue(owlready2.isinstance_python(dynamic_ontology_concept_holder.ontology_concept, owlready2.Thing))

    @unittest.skipUnless(owlready2, 'Owlready2 is required')
    def test_loaded_ontologies(self):
        self.assertIsNotNone(self.main_ontology)
        self.assertTrue(self.main_ontology.loaded)
        if self.ontology_manager.main_ontology_iri is SOMA_ONTOLOGY_IRI or \
                self.ontology_manager.main_ontology_iri is SOMA_HOME_ONTOLOGY_IRI:
            self.assertIsNotNone(self.soma)
            self.assertTrue(self.soma.loaded)
            self.assertIsNotNone(self.dul)
            self.assertTrue(self.dul.loaded)

    @unittest.skipUnless(owlready2, 'Owlready2 is required')
    def test_ontology_concept_class_dynamic_creation(self):
        dynamic_ontology_concept_class = self.ontology_manager.create_ontology_concept_class('DynamicOntologyConcept')
        self.assertIsNotNone(dynamic_ontology_concept_class)
        self.assertEqual(dynamic_ontology_concept_class.namespace, self.main_ontology)
        self.assertIs(dynamic_ontology_concept_class, self.main_ontology.DynamicOntologyConcept)
        self.assertIs(issubclass(dynamic_ontology_concept_class, owlready2.Thing), True)
        dynamic_ontology_concept = dynamic_ontology_concept_class(name='dynamic_ontology_concept2',
                                                                  namespace=self.main_ontology)
        self.assertTrue(owlready2.isinstance_python(dynamic_ontology_concept, owlready2.Thing))

    @unittest.skipUnless(owlready2, 'Owlready2 is required')
    def test_ontology_triple_classes_dynamic_creation(self):
        # Test dynamic triple classes creation without inheritance from existing parent ontology classes
        self.assertTrue(self.ontology_manager.create_ontology_triple_classes(subject_class_name="OntologySubject",
                                                                             object_class_name="OntologyObject",
                                                                             predicate_class_name="predicate",
                                                                             inverse_predicate_class_name="inverse_predicate"))

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
        self.assertTrue(
            self.ontology_manager.create_ontology_triple_classes(
                ontology_subject_parent_class=self.soma.Container if self.soma else None,
                subject_class_name="OntologyPlaceHolderObject",
                ontology_object_parent_class=self.dul.PhysicalObject
                if self.dul else None,
                object_class_name="OntologyHandheldObject",
                predicate_class_name=PLACEABLE_ON_PREDICATE_NAME,
                inverse_predicate_class_name=HOLD_OBJ_PREDICATE_NAME,
                ontology_property_parent_class=self.soma.affordsBearer
                if self.soma else None,
                ontology_inverse_property_parent_class=self.soma.isBearerAffordedBy
                if self.soma else None))

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

    @unittest.skipUnless(owlready2, 'Owlready2 is required')
    def test_ontology_class_destruction(self):
        concept_class_name = 'DynamicOntologyConcept'
        dynamic_ontology_concept_class = self.ontology_manager.create_ontology_concept_class(concept_class_name)
        OntologyConceptHolder(dynamic_ontology_concept_class(name='dynamic_ontology_concept3',
                                                             namespace=self.main_ontology))

        self.ontology_manager.destroy_ontology_class(dynamic_ontology_concept_class)
        self.assertIsNone(self.ontology_manager.get_ontology_class(concept_class_name))
        self.assertFalse(OntologyConceptHolderStore().get_ontology_concepts_by_class(dynamic_ontology_concept_class))

    @unittest.skipUnless(owlready2, 'Owlready2 is required')
    @unittest.skipUnless(java_runtime_installed, 'Java runtime is required')
    def test_ontology_reasoning(self):
        REASONING_TEST_ONTOLOGY_IRI = f"reasoning_test{ONTOLOGY_OWL_FILE_EXTENSION}"
        ENTITY_CONCEPT_NAME = "Entity"
        CAN_TRANSPORT_PREDICATE_NAME = "can_transport"
        TRANSPORTABLE_BY_PREDICATE_NAME = "transportable_by"
        CORESIDE_PREDICATE_NAME = "coreside"

        # Create a test world (with memory SQL backend) for reasoning
        reasoning_world = self.ontology_manager.create_ontology_world()
        reasoning_ontology = reasoning_world.get_ontology(REASONING_TEST_ONTOLOGY_IRI)

        # Create Entity class & types of relations among its instances
        self.assertTrue(self.ontology_manager.create_ontology_triple_classes(subject_class_name=ENTITY_CONCEPT_NAME,
                                                                             object_class_name=ENTITY_CONCEPT_NAME,
                                                                             predicate_class_name=CAN_TRANSPORT_PREDICATE_NAME,
                                                                             inverse_predicate_class_name=TRANSPORTABLE_BY_PREDICATE_NAME,
                                                                             ontology_property_parent_class=owlready2.ObjectProperty,
                                                                             ontology_inverse_property_parent_class=owlready2.ObjectProperty,
                                                                             ontology=reasoning_ontology))

        self.assertTrue(self.ontology_manager.create_ontology_triple_classes(subject_class_name=ENTITY_CONCEPT_NAME,
                                                                             object_class_name=ENTITY_CONCEPT_NAME,
                                                                             predicate_class_name=CORESIDE_PREDICATE_NAME,
                                                                             ontology_property_parent_class=owlready2.ObjectProperty,
                                                                             ontology=reasoning_ontology))

        # Define rules for `transportability` & `co-residence` in [reasoning_ontology]
        with reasoning_ontology:
            def can_transport_itself(a: reasoning_ontology.Entity) -> bool:
                return a in a.can_transport

            def can_transport_entity(a: reasoning_ontology.Entity, b: reasoning_ontology.Entity) -> bool:
                return b in a.can_transport

            def can_be_transported_by(a: reasoning_ontology.Entity, b: reasoning_ontology.Entity) -> bool:
                return b in a.transportable_by

            def coresidents(a: reasoning_ontology.Entity, b: reasoning_ontology.Entity) -> bool:
                return b in a.coreside

            # Rule1: Transitivity of transportability
            self.ontology_manager.create_rule_transitivity(ontology_concept_class_name=ENTITY_CONCEPT_NAME,
                                                           predicate_name=CAN_TRANSPORT_PREDICATE_NAME,
                                                           ontology=reasoning_ontology)

            # Rule2: reflexivity of transportability
            self.ontology_manager.create_rule_reflexivity(ontology_concept_class_name=ENTITY_CONCEPT_NAME,
                                                          predicate_name=CAN_TRANSPORT_PREDICATE_NAME,
                                                          ontology=reasoning_ontology)

            # Rule3 & 4: Symmetry & Transitivity of co-residence
            self.ontology_manager.create_rule_transitivity(ontology_concept_class_name=ENTITY_CONCEPT_NAME,
                                                           predicate_name=CORESIDE_PREDICATE_NAME,
                                                           ontology=reasoning_ontology)
            self.ontology_manager.create_rule_symmetry(ontology_concept_class_name=ENTITY_CONCEPT_NAME,
                                                       predicate_name=CORESIDE_PREDICATE_NAME,
                                                       ontology=reasoning_ontology)

            # Create entities
            entities = [reasoning_ontology.Entity(name=f"e{i}") for i in range(3)]
            entities[2].can_transport.append(entities[1])
            entities[1].can_transport.append(entities[0])
            entities[0].coreside.append(entities[1])
            entities[0].coreside.append(entities[2])

            # Reason on [reasoning_world]
            self.ontology_manager.reason(world=reasoning_world)

            # Test reflexivity
            for entity in entities:
                self.assertTrue(can_transport_itself(entity))

            # Test transitivity
            self.assertTrue(can_transport_entity(entities[2], entities[0]))
            self.assertTrue(can_be_transported_by(entities[0], entities[2]))

            # Test symmetry
            entities_num = len(entities)
            for i in range(entities_num):
                for j in range(entities_num):
                    if i != j:
                        self.assertTrue(coresidents(entities[i], entities[j]))

    @unittest.skipUnless(owlready2, 'Owlready2 is required')
    def test_ontology_save(self):
        save_dir = self.ontology_manager.get_main_ontology_dir()
        owl_filepath = f"{save_dir}/{Path(self.ontology_manager.main_ontology_iri).stem}{ONTOLOGY_OWL_FILE_EXTENSION}"
        self.assertTrue(self.ontology_manager.save(owl_filepath))
        self.assertTrue(Path(owl_filepath).is_file())
        sql_backend = self.ontology_manager.main_ontology_sql_backend
        if sql_backend != ONTOLOGY_SQL_IN_MEMORY_BACKEND:
            self.assertTrue(Path(sql_backend).is_file())

if __name__ == '__main__':
    unittest.main()
