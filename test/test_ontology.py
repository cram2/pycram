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
        save_dir = Path(f"{Path.home()}/ontologies")
        owl_filepath = f"{save_dir}/{Path(cls.ontology_manager.main_ontology_iri).stem}.owl"
        sql_filepath = f"{save_dir}/{Path(owl_filepath).stem}.sqlite3"
        os.remove(owl_filepath)
        cls.remove_sql_file(sql_filepath)

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

    def test_ontology_world(self):
        if owlready2:
            self.assertIsNotNone(self.ontology_manager.main_ontology_world)
            extra_world = self.ontology_manager.create_ontology_world()
            self.assertIsNotNone(extra_world)
            extra_world_sql_filepath = f"{onto_path[0]}/extra_world.sqlite3"
            extra_world.save(file=extra_world_sql_filepath)
            world_saved_to_sql = Path(extra_world_sql_filepath).is_file()
            self.remove_sql_file(extra_world_sql_filepath)
            self.assertTrue(world_saved_to_sql)

    def test_ontology_concept_holder(self):
        if not owlready2:
            return
        dynamic_ontology_concept_class = self.ontology_manager.create_ontology_concept_class('DynamicOntologyConcept')
        dynamic_ontology_concept_holder = OntologyConceptHolder(
            dynamic_ontology_concept_class(name='dynamic_ontology_concept1',
                                           namespace=self.main_ontology))
        self.assertTrue(owlready2.isinstance_python(dynamic_ontology_concept_holder.ontology_concept, owlready2.Thing))

    def test_loaded_ontologies(self):
        if not owlready2:
            return
        self.assertIsNotNone(self.main_ontology)
        self.assertTrue(self.main_ontology.loaded)
        if self.ontology_manager.main_ontology_iri is SOMA_ONTOLOGY_IRI:
            self.assertIsNotNone(self.soma)
            self.assertTrue(self.soma.loaded)
            self.assertIsNotNone(self.dul)
            self.assertTrue(self.dul.loaded)

    def test_ontology_concept_class_dynamic_creation(self):
        if not owlready2:
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
        if not owlready2:
            return
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
            self.ontology_manager.create_ontology_triple_classes(ontology_subject_parent_class=self.soma.Container if self.soma else None,
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

    def test_ontology_class_destruction(self):
        if not owlready2:
            return
        concept_class_name = 'DynamicOntologyConcept'
        dynamic_ontology_concept_class = self.ontology_manager.create_ontology_concept_class(concept_class_name)
        OntologyConceptHolder(dynamic_ontology_concept_class(name='dynamic_ontology_concept3',
                                                             namespace=self.main_ontology))

        self.ontology_manager.destroy_ontology_class(dynamic_ontology_concept_class)
        self.assertIsNone(self.ontology_manager.get_ontology_class(concept_class_name))
        self.assertFalse(OntologyConceptHolderStore().get_ontology_concepts_by_class(dynamic_ontology_concept_class))

    def test_ontology_reasoning(self):
        if not owlready2:
            return

        REASONING_TEST_ONTOLOGY_IRI = "reasoning_test.owl"
        ENTITY_CONCEPT_NAME = "Entity"
        CAN_TRANSPORT_PREDICATE_NAME = "can_transport"
        TRANSPORTABLE_BY_PREDICATE_NAME = "transportable_by"
        CORESIDE_PREDICATE_NAME = "coreside"

        # Create a test world for reasoning
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

        # Define rules for "bigger_than" in [reasoning_ontology]
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
            self.remove_sql_file(sql_filepath=f"{onto_path[0]}/{Path(reasoning_ontology.name).stem}.sqlite3")

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

    def test_ontology_save(self):
        if not owlready2:
            return

        save_dir = Path(f"{Path.home()}/ontologies")
        save_dir.mkdir(parents=True, exist_ok=True)
        owl_filepath = f"{save_dir}/{Path(self.ontology_manager.main_ontology_iri).stem}.owl"
        sql_filepath = f"{save_dir}/{Path(owl_filepath).stem}.sqlite3"
        self.ontology_manager.save(owl_filepath)
        self.assertTrue(Path(owl_filepath).is_file())
        self.assertTrue(Path(sql_filepath).is_file())


if __name__ == '__main__':
    unittest.main()
