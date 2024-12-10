from .dependencies import *


class precedes(BaseProperty):
    """
    A relation between entities, expressing a 'sequence' schema. 
    E.g. 'year 1999 precedes 2000', 'deciding what coffee to use' precedes 'preparing coffee', 'World War II follows World War I', 'in the Milan to Rome autoroute, Bologna precedes Florence', etc.
    It can then be used between tasks, processes, time intervals, spatially locate objects, situations, etc. 
    Subproperties can be defined in order to distinguish the different uses.
    """
    
class defines(BaseProperty):
    """
    A relation between a Description and a Concept, e.g. a Workflow for a governmental Organization defines the Role 'officer', or 'the Italian Traffic Law defines the role Vehicle'.
    """
    
class definesTask(BaseProperty):
    """
    A relation between a description and a task, e.g. the recipe for a cake defines the task 'boil'.
    """
    
class isDescribedBy(BaseProperty):
    """
    The relation between an Entity and a Description: a Description gives a unity to a Collection of parts (the components), or constituents, by assigning a Role to each of them in the context of a whole Object (the system).
    A same Entity can be given different descriptions, for example, an old cradle can be given a unifying Description based on the original aesthetic design, the functionality it was built for, or a new aesthetic functionality in which it can be used as a flower pot.
    """
    
class associatedWith(BaseProperty):
    """
    A catch-all object property, useful for alignment and querying purposes.
    It is declared as both transitive and symmetric, in order to reason an a maximal closure of associations between individuals.
    """
    
class follows(BaseProperty):
    """
    A relation between entities, expressing a 'sequence' schema. 
    E.g. 'year 2000 follows 1999', 'preparing coffee' follows 'deciding what coffee to use', 'II World War follows I World War', etc. 
    It can be used between tasks, processes or time intervals, and subproperties would fit best in order to distinguish the different uses.
    """
    
class isEventIncludedIn(BaseProperty):
    ...
    
class overlaps(BaseProperty):
    """
    A schematic relation between any entities, e.g. 'the chest region overlaps with the abdomen region', 'my spoken words overlap with hers', 'the time of my leave overlaps with the time of your arrival', 'fibromyalgia overlaps with other conditions'.
    Subproperties and restrictions can be used to specialize overlaps for objects, events, time intervals, etc.
    """
    
class isLocationOf(BaseProperty):
    """
    A generic, relative localization, holding between any entities. E.g. 'Rome is the seat of the Pope', 'the liver is the location of the tumor'.
    For 'absolute' locations, see SpaceRegion
    """
    
class definesRole(BaseProperty):
    """
    A relation between a description and a role, e.g. the recipe for a cake defines the role 'ingredient'.
    """
    
class describes(BaseProperty):
    """
    The relation between a Description and an Entity : a Description gives a unity to a Collection of parts (the components), or constituents, by assigning a Role to each of them in the context of a whole Object (the system).
    A same Entity can be given different descriptions, for example, an old cradle can be given a unifying Description based on the original aesthetic design, the functionality it was built for, or a new aesthetic functionality in which it can be used as a flower pot.
    """
    
class includesEvent(BaseProperty):
    """
    A relation between situations and events, e.g. 'this morning I've prepared my coffee and had my fingers burnt' (i.e.: the preparation of my coffee this morning included a burning of my fingers).
    """
    
class hasMember(BaseProperty):
    """
    A relation between collections and entities, e.g. 'my collection of saxophones includes an old Adolphe Sax original alto' (i.e. my collection has member an Adolphe Sax alto).
    """
    
class hasConstituent(BaseProperty):
    """
    'Constituency' depends on some layering of  the world described by the ontology. For example, scientific granularities (e.g. body-organ-tissue-cell) or ontological 'strata' (e.g. social-mental-biological-physical) are  typical layerings. 
    Intuitively, a constituent is a part belonging to a lower layer. Since layering is actually a partition of the world described by the ontology, constituents are not properly classified as parts, although this kinship can be intuitive for common sense.
    A desirable advantage of this distinction is that we are able to talk e.g. of physical constituents of non-physical objects (e.g. systems), while this is not possible in terms of parts.
    Example of are the persons constituting a social system, the molecules constituting a person, the atoms constituting a river, etc. 
    In all these examples, we notice a typical discontinuity between the constituted and the constituent object: e.g. a social system is conceptualized at a different layer from the persons that constitute it, a person is conceptualized at a different layer from the molecules that constitute them, and a river is conceptualized at a different layer from the atoms that constitute it.
    """
    
class hasRegion(BaseProperty):
    """
    A relation between entities and regions, e.g. 'the number of wheels of that truck is 12', 'the time of the experiment is August 9th, 2004', 'the whale has been localized at 34 degrees E, 20 degrees S'.
    """
    
class hasPart(BaseProperty):
    """
    A schematic relation between any entities, e.g. 'the human body has a brain as part', '20th century contains year 1923', 'World War II includes the Pearl Harbour event'.
    
    Parthood should assume the basic properties of mereology: transitivity, antisymmetry, and reflexivity (propert Parthood of course misses reflexivity). 
    However, antisymmetry is not supported in OWL2 explicitly, therefore DUL has to adopt one of two patterns:
    1) dropping asymmetry axioms, while granting reflexivity: this means that symmetry is not enforced, but permitted for the case of reflexivity. Of course, in this way we cannot prevent symmetric usages of hasPart;
    2) dropping the reflexivity axiom, and enforce asymmetry: in this case, we would prevent all symmetric usages, but we loose the possibility of enforcing reflexivity, which is commonsensical in parthood.
    In DUL, we adopt pattern #1 for partOf, and pattern #2 for properPartOf, which seems a good approximation: due to the lack of inheritance of property characteristics, each asymmetric hasPropertPart assertion would also be a reflexive hasPart assertion (reflexive reduction design pattern).
    
    Subproperties and restrictions can be used to specialize hasPart for objects, events, etc.
    """
    
class hasQuality(BaseProperty):
    """
    A relation between entities and qualities, e.g. 'Dmitri's skin is yellowish'.
    """
    
class hasParameter(BaseProperty):
    """
    A Concept can have a Parameter that constrains the attributes that a classified Entity can have in a certain Situation, e.g. a 4WheelDriver Role definedIn the ItalianTrafficLaw has a MinimumAge parameter on the Amount 16.
    """
    
class hasComponent(BaseProperty):
    """
    The hasProperPart relation without transitivity, holding between an Object (the system) and another (the component), and assuming a Design that structures the Object.
    """
    
class directlyPrecedes(BaseProperty):
    """
    The intransitive precedes relation. For example, Monday directly precedes Tuesday. Directness of precedence depends on the designer conceptualization.
    """
    
class directlyFollows(BaseProperty):
    """
    The intransitive follows relation. For example, Wednesday directly precedes Thursday. Directness of precedence depends on the designer conceptualization.
    """
    
class isRelatedToConcept(BaseProperty):
    """
    Any relation between concepts, e.g. superordinated, conceptual parthood, having a parameter, having a task, superordination, etc.
    """
    
class involvesAgent(BaseProperty):
    """
    Agent participation.
    """
    
class includesObject(BaseProperty):
    """
    A relation between situations and objects, e.g. 'this morning I've prepared my coffee and had my fingers burnt' (i.e.: the preparation of my coffee this morning included me).
    """
    
class isReferenceOf(BaseProperty):
    """
    A relation between information objects and any Entity (including information objects). It can be used to talk about e.g. entities are references of proper nouns: the proper noun 'Leonardo da Vinci' isAbout the Person Leonardo da Vinci; as well as to talk about sets of entities that can be described by a common noun: the common noun 'person' isAbout the set of all persons in a domain of discourse, which can be represented in DOLCE-Ultralite as an individual of the class: Collection .
    The isReferenceOf relation is irreflexive, differently from its inverse isAbout.
    """
    
class isSettingFor(BaseProperty):
    """
    A relation between situations and entities, e.g. 'this morning I've prepared my coffee with a new fantastic Arabica', i.e.: the preparation of my coffee this morning is the setting for (an amount of) a new fantastic Arabica.
    """
    
class hasParticipant(BaseProperty):
    """
    A relation between an object and a process, e.g. 'John took part in the discussion', 'a large mass of snow fell during the avalanche', or 'a cook, some sugar, flour, etc. are all present in the cooking of a cake'.
    """
    
class isRegionFor(BaseProperty):
    """
    A relation between entities and regions, e.g. 'the color of my car is red'.
    """
    
class isParticipantIn(BaseProperty):
    """
    A relation between an object and a process, e.g. 'John took part in the discussion', 'a large mass of snow fell during the avalanche', or 'a cook, some sugar, flour, etc. are all present in the cooking of a cake'.
    """
    
class isRoleDefinedIn(BaseProperty):
    """
    A relation between a description and a role, e.g. the role 'Ingredient' is defined in the recipe for a cake.
    """
    
class isConstituentOf(BaseProperty):
    """
    'Constituency' depends on some layering of  the world described by the ontology. For example, scientific granularities (e.g. body-organ-tissue-cell) or ontological 'strata' (e.g. social-mental-biological-physical) are  typical layerings. 
    Intuitively, a constituent is a part belonging to a lower layer. Since layering is actually a partition of the world described by the ontology, constituents are not properly classified as parts, although this kinship can be intuitive for common sense.
    A desirable advantage of this distinction is that we are able to talk e.g. of physical constituents of non-physical objects (e.g. systems), while this is not possible in terms of parts.
    Example of are the persons constituting a social system, the molecules constituting a person, the atoms constituting a river, etc. 
    In all these examples, we notice a typical discontinuity between the constituted and the constituent object: e.g. a social system is conceptualized at a different layer from the persons that constitute it, a person is conceptualized at a different layer from the molecules that constitute them, and a river is conceptualized at a different layer from the atoms that constitute it.
    """
    
class isQualityOf(BaseProperty):
    """
    A relation between entities and qualities, e.g. 'Dmitri's skin is yellowish'.
    """
    
class isObjectIncludedIn(BaseProperty):
    ...
    
class isDefinedIn(BaseProperty):
    """
    A relation between a Description and a Concept, e.g. a Workflow for a governmental Organization defines the Role 'officer', or 'the Italian Traffic Law defines the role Vehicle'.
    """
    
class isRealizedBy(BaseProperty):
    """
    A relation between an information realization and an information object, e.g. the paper copy of the Italian Constitution realizes the text of the Constitution.
    """
    
class hasRole(BaseProperty):
    """
    A relation between an object and a role, e.g. the person 'John' has role 'student'.
    """
    
class isRoleOf(BaseProperty):
    """
    A relation between an object and a role, e.g. 'student' is the role of 'John'.
    """
    
class realizes(BaseProperty):
    """
    A relation between an information realization and an information object, e.g. the paper copy of the Italian Constitution realizes the text of the Constitution.
    """
    
class isParameterFor(BaseProperty):
    """
    A Concept can have a Parameter that constrains the attributes that a classified Entity can have in a certain Situation, e.g. a 4WheelDriver Role definedIn the ItalianTrafficLaw has a MinimumAge parameter on the Amount 16.
    """
    
class hasTask(BaseProperty):
    """
    A relation between roles and tasks, e.g. 'students have the duty of giving exams' (i.e. the Role 'student' hasTask the Task 'giving exams').
    """
    
class hasLocation(BaseProperty):
    """
    A generic, relative spatial location, holding between any entities. E.g. 'the cat is on the mat', 'Omar is in Samarcanda', 'the wound is close to the femural artery'.
    For 'absolute' locations, see SpaceRegion
    """
    
class isComponentOf(BaseProperty):
    """
    The asymmetric isProperPartOf relation without transitivity, holding between an Object (the system) and another (the component), and assuming a Design that structures the Object.
    """
    
class isClassifiedBy(BaseProperty):
    """
    A relation between a Concept and an Entity, e.g. 'John is considered a typical rude man'; your last concert constitutes the achievement of a lifetime; '20-year-old means she's mature enough'.
    """
    
class classifies(BaseProperty):
    """
    A relation between a Concept and an Entity, e.g. the Role 'student' classifies a Person 'John'.
    """
    
class isAbout(BaseProperty):
    """
    A relation between an information object and an Entity (including information objects). It can be used to talk about entities that are references of proper nouns: the proper noun 'Leonardo da Vinci' isAbout the Person Leonardo da Vinci; as well as to talk about sets of entities that can be described by a common noun: the common noun 'person' isAbout the set of all persons in a domain of discourse, which can be represented in DOLCE-Ultralite as an individual of the class: dul:Collection.
    A specific sentence may use common nouns with either a singular or plural reference, or it can even refer to all possible references (e.g. in a lexicographic definition): all those uses are kinds of aboutness.
    
    The isAbout relation is sometimes considered as reflexive, however this is semiotically inaccurate, because information can be about itself ('de dicto' usage, as in 'John is four character long'), but it is typically about something else ('de re' usage, as in 'John loves Mary').
    If a reflexivity exists in general, it rather concerns its realisation, which is always associated with an event, e.g. an utterance, which makes the information denoting itself, besides its aboutness. This is implemented in DUL with the dul:realizesSelfInformation property, which is used with local reflexivity in the dul:InformationRealization class.
    """
    
class hasSetting(BaseProperty):
    """
    A relation between entities and situations, e.g. 'this morning I've prepared my coffee with a new fantastic Arabica', i.e.: (an amount of) a new fantastic Arabica hasSetting the preparation of my coffee this morning.
    """
    
class isTaskDefinedIn(BaseProperty):
    """
    A relation between a description and a task, e.g. the task 'boil' is defined in a recipe for a cake.
    """
    
class hasCommonBoundary(BaseProperty):
    """
    A relation to encode either formal or informal characterizations of 'boundaries' common to two different entities: an Event that ends when another begins, two abstract regions that have a common topological boundary, two objects that are said to be 'in contact' from a commonsense perspective, etc.
    """
    
class isTaskOf(BaseProperty):
    """
    A relation between roles and tasks, e.g. 'students have the duty of giving exams' (i.e. the Role 'student' hasTask the Task 'giving exams').
    """
    
class hasPostcondition(BaseProperty):
    """
    Direct succession applied to situations. 
    E.g., 'A postcondition of our Plan is to have things settled'.
    """
    
class hasPrecondition(BaseProperty):
    """
    Direct precedence applied to situations. 
    E.g., 'A precondition to declare war against a foreign country is claiming to find nuclear weapons in it'.
    """
    
class actsFor(BaseProperty):
    """
    The relation holding between any Agent, and a SocialAgent. In principle, a SocialAgent requires at least one PhysicalAgent in order to act, but this dependency can be 'delegated'; e.g. a university can be acted for by a department, which on its turm is acted for by physical agents.
    """
    
class executesTask(BaseProperty):
    """
    A relation between an action and a task, e.g. 'putting some water in a pot and putting the pot on a fire until the water starts bubbling' executes the task 'boiling'.
    """
    
class expresses(BaseProperty):
    """
    A relation between an InformationObject and a 'meaning', generalized here as a 'SocialObject'. For example: 'A Beehive is a structure in which bees are kept, typically in the form of a dome or box.' (Oxford dictionary)'; 'the term Beehive expresses the concept Beehive in my apiculture ontology'.
    The intuition for 'meaning' is intended to be very broad. A separate, large comment is included for those who want to investigate more on what kind of meaning can be represented in what form.
    This is a large comment field for those who want to investigate the different uses of the 'expresses' relation for modeling different approaches to meaning characterization and modeling.
    For example, in all these cases, some aspect of meaning is involved:
    
    - Beehive means "a structure in which bees are kept, typically in the form of a dome or box." (Oxford dictionary)
    - 'Beehive' is a synonym in noun synset 09218159 "beehive|hive" (WordNet)
    - 'the term Beehive can be interpreted as the fact of 'being a beehive', i.e. a relation that holds for concepts such as Bee, Honey, Hosting, etc.'
    - 'the text of Italian apiculture regulation expresses a rule by which beehives should be kept at least one kilometer away from inhabited areas'
    - 'the term Beehive expresses the concept Beehive'
    - ''Beehive' for apiculturists does not express the same meaning as for, say, fishermen'
    - 'Your meaning of 'Beautiful' does not seem to fit mine'
    - ''Beehive' is formally interpreted as the set of all beehives'
    - 'from the term 'Beehive', we can build a vector space of statistically significant cooccurring terms in the documents that contain it'
    - the lexeme 'Belly' expresses the role 'Body_Part' in the frame 'ObservableBodyParts' (FrameNet)
    
    As the examples suggest, the 'meaning of meaning' is dependent on the background approach/theory that one assumes. One can hardly make a summary of the too many approaches and theories of meaning, therefore this relation is maybe the most controversial and difficult to explain; normally, in such cases it would be better to give up formalizing. 
    However, the usefulness of having a 'semantic abstraction' in modeling information objects is so high (e.g. for the semantic web, interoperability, reengineering, etc.), that we accept this challenging task, although without taking any particular position in the debate. 
    We provide here some examples, which we want to generalize upon when using the 'expresses' relation to model semantic aspects of social reality.
    
    In the most common approach, lexicographers that write dictionaries, glossaries, etc. assume that the meaning of a term is a paraphrase (or 'gloss', or 'definition'). 
    Another approach is provided by concept schemes like thesauri and lexicons, which assume that the meaning of a term is a 'concept', encoded as a 'lemma', 'synset', or 'descriptor'.
    Still another approach is that of psychologists and cognitive scientists, which often assume that the meaning of an information object is a concept encoded in the mind or cognitive system of an agent. 
    A radically different approach is taken by social scientists and semioticians, who usually assume that meanings of an information object are spread across the communication practices in which members of a community use that object.
    Another approach that tackles the distributed nature of meaning is assumed by geometrical models of semantics, which assume that the meaning of an InformationObject (e.g. a word) results from the set of informational contexts (e.g. within texts) in which that object is used similarly.
    The logical approach to meaning is still different, since it assumes that the meaning of e.g. a term is equivalent to the set of individuals that the term can be applied to; for example, the meaning of 'Ali' is e.g. an individual person called Ali, the meaning of 'Airplane' is e.g. the set of airplanes, etc. 
    Finally, an approach taken by structuralist linguistics and frame semantics is that a meaning is the relational context in which an information object can be applied; for example, a meaning of 'Airplane' is situated e.g. in the context ('frame') of passenger airline flights.
    
    These different approaches are not necessarily conflicting, and they mostly talk about different aspects of so-called 'semantics'. They can be summarized and modelled within DOLCE-Ultralite as follows (notice that such list is far from exhaustive):
    
    (1) Informal meaning (as for linguistic or commonsense semantics: a distinction is assumed between (informal) meaning and reference; see isAbout for an alternative pattern on reference)
    	- Paraphrase meaning (as for lexicographic semantics). Here it is modelled as the expresses relation between instances of InformationObject and different instances of InformationObject that act as 'paraphrases'
    	- Conceptual meaning (as for 'concept scheme' semantics). Here it is modelled as the expresses relation between instances of InformationObject and instances of Concept
    	- Relational meaning (as for frame semantics). Here it is modelled as the expresses relation between instances of InformationObject and instances of Description
    	- Cognitive meaning (as for 'psychological' semantics). Here it is modelled as the expresses relation between any instance of InformationObject and any different instance of InformationObject that isRealizedBy a mental, cognitive or neural state (depending on which theory of mind is assumed). Such states can be considered here as instances of Process (occurring in the mind, cognitive system, or neural system of an agent)
    	- Cultural meaning (as for 'social science' semantics). Here it is modelled as the expresses relation between instances of InformationObject and instances of SocialObject (institutions, cultural paradigms, norms, social practices, etc.)
    	- Distributional meaning (as for geometrical models of meaning). Here it is modelled as the expresses relation between any instance of InformationObject and any different instance of InformationObject that isFormallyRepresentedIn some (geometrical) Region (e.g. a vector space)
    
    (2) Formal meaning (as for logic and formal semantics: no distinction is assumed between informal meaning and reference, therefore between 'expresses' and 'isAbout', which can be used interchangeably)
    	- Object-level formal meaning (as in the traditional first-order logic semantics). Here it is modelled as the expresses relation between an instance of InformationObject and an instance of Collection that isGroundingFor (in most cases) a Set; isGroundingFor is defined in the ontology: http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl
    	- Modal formal meaning (as in possible-world semantics). Here it is modelled as the expresses relation between an instance of InformationObject and an instance of Collection that isGroundingFor a Set, and which isPartOf some different instance of Collection that isGroundingFor a PossibleWorld
    
    This is only a first step to provide a framework, in which one can model different aspects of meaning. A more developed ontology should approach the problem of integrating the different uses of 'expresses', so that different theories, resources, methods can interoperate.
    """
    
class isExecutedIn(BaseProperty):
    """
    A relation between an action and a task, e.g. 'putting some water in a pot and putting the pot on a fire until the water starts bubbling' executes the task 'boiling'.
    """
    
class isExpressedBy(BaseProperty):
    """
    A relation between a dul:SocialObject (the 'meaning') and a dul:InformationObject (the 'expression'). 
    For example: 'A Beehive is a structure in which bees are kept, typically in the form of a dome or box.' (Oxford dictionary)'; 'the term Beehive expresses the concept Beehive in my apiculture ontology'.
    The intuition for 'meaning' is intended to be very broad. A separate, large comment is included in the encoding of 'expresses', for those who want to investigate more on what kind of meaning can be represented in what form.
    """
    
class isPartOf(BaseProperty):
    """
    A relation between any entities, e.g. 'brain is a part of the human body'. See dul:hasPart for additional documentation.
    """
    
class satisfies(BaseProperty):
    """
    A relation between a Situation and a Description, e.g. the execution of a Plan satisfies that plan.
    """
    
class realizesSelfInformation(BaseProperty):
    """
    This relation is a workaround to enable local reflexivity axioms (Self) working with non-simple properties; in this case, dul:realizesInformation About.
    """
    
class actsThrough(BaseProperty):
    """
    The relation holding between a PhysicalAgent and a SocialAgent. In principle, a SocialAgent requires at least one PhysicalAgent in order to act, but this dependency can be 'delegated', e.g. a university can be acted for by a department, which is acted for by physical agents. AKA isActedBy
    """
    
class characterizes(BaseProperty):
    """
    A relation between concepts and collections, where a Concept is said to characterize a Collection; it corresponds to a link between the (reified) intensional and extensional interpretations of a _proper subset of_ a (reified) class. This is different from covers, because it refers to an interpretation the entire reified class.
    E.g. the collection of vintage saxophones is characterized by the Concept 'manufactured by hand', while it gets covered by the Concept 'Saxophone' with the Parameter 'Vintage'.
    """
    
class isCharacterizedBy(BaseProperty):
    ...
    
class conceptualizes(BaseProperty):
    """
    A relation stating that an Agent is internally representing a SocialObject: situations, descriptions, concepts, etc. E.g., 'John believes in the conspiracy theory'; 'Niels Bohr created the solar-system metaphor for the atomic theory'; 'Jacques assumes all swans are white'; 'the task force members share the attack plan'.
    Conceptualizations can be distinguished into different forms, primarily based on the type of SocialObject that is conceptualized. Descriptions and concepts can be 'assumed', situations can be 'believed' or 'known', plans can be 'adopted', etc. (see ontology: http://www.ontologydesignpatterns.org/ont/dul/Conceptualization.owl.
    """
    
class isConceptualizedBy(BaseProperty):
    """
    A relation stating that an Agent is internally representing a Description . E.g., 'John believes in the conspiracy theory'; 'Niels Bohr created a solar-system metaphor for his atomic theory'; 'Jacques assumes all swans are white'; 'the task force shares the attack plan'.
    """
    
class concretelyExpresses(BaseProperty):
    """
    A relation between an InformationRealization and a Description, e.g. 'the printout of the Italian Constitution concretelyExpresses the Italian Constitution'. It should be supplied also with a rule stating that the InformationRealization realizes an InformationObject that expresses the Description
    """
    
class isConcretelyExpressedBy(BaseProperty):
    """
    A relation between an InformationRealization and a Description, e.g. 'the printout of the Italian Constitution concretelyExpresses the Italian Constitution'. It should be supplied also with a rule stating that the InformationRealization realizes an InformationObject that expresses the Description
    """
    
class coparticipatesWith(BaseProperty):
    """
    A relation between two objects participating in a same Event; e.g., 'Vitas and Jimmy are playing tennis'.
    """
    
class covers(BaseProperty):
    """
    A relation between concepts and collections, where a Concept is said to cover a Collection; it corresponds to a link between the (reified) intensional and extensional interpretations of a (reified) class.
    E.g. the collection of vintage saxophones is covered by the Concept 'Saxophone' with the Parameter 'Vintage'.
    """
    
class isCoveredBy(BaseProperty):
    """
    A relation between concepts and collections, where a Concept is said to cover a Collection; it corresponds to a link between the (reified) intensional and extensional interpretations of a (reified) class.
    E.g. the collection of vintage saxophones is covered by the Concept 'Saxophone' with the Parameter 'Vintage'.
    """
    
class usesConcept(BaseProperty):
    """
    A generic relation holding between a Description and a Concept. In order to be used, a Concept must be previously definedIn another Description. This last condition cannot be encoded for object properties in OWL.
    """
    
class expands(BaseProperty):
    """
    A partial order relation that holds between descriptions. It represents the proper part relation between a description and another description featuring the same properties as the former, with at least one additional one.
    Descriptions can be expanded either by adding other descriptions as parts, or by refining concepts that are used by them. 
    An 'intention' to expand must be present (unless purely formal theories are considered, but even in this case a criterion of relevance is usually active).
    """
    
class isRelatedToDescription(BaseProperty):
    """
    Any relation between descriptions.
    """
    
class isExpandedIn(BaseProperty):
    """
    A partial order relation that holds between descriptions. It represents the proper part relation between a description and another description featuring the same properties as the former, with at least one additional one.
    Descriptions can be expanded either by adding other descriptions as parts, or by refining concepts that are used by them. 
    An 'intention' to expand must be present (unless purely formal theories are considered, but even in this case a criterion of relevance is usually active).
    """
    
class expressesConcept(BaseProperty):
    """
    A relation between an InformationObject and a Concept , e.g. the term "dog" expresses the Concept "dog". For expressing a relational meaning, see the more general object property: expresses
    """
    
class isConceptExpressedBy(BaseProperty):
    """
    A relation between an InformationObject and a Concept , e.g. the term "dog" expresses the Concept "dog". For expressing a relational meaning, see the more general object property: expresses
    """
    
class farFrom(BaseProperty):
    """
    Generic distance relation between any Entity(s). E.g. Rome is far from Beijing, astronomy is far from necromancy.
    """
    
class hasProperPart(BaseProperty):
    """
    Asymmetric (so including irreflexive) parthood.
    """
    
class hasConstraint(BaseProperty):
    """
    A relation between parameters and entities. It allows to assert generic constraints (encoded as parameters), e.g. MinimumAgeForDriving isConstraintFor John (where John is a legal subject under the TrafficLaw).
    The intended semantics (not expressible in OWL) is that a Parameter isParameterFor a Concept that classifies an Entity; moreover, it entails that a Parameter parametrizes a Region that isRegionFor that Entity.
    """
    
class isConstraintFor(BaseProperty):
    """
    A relation between parameters and entities. It allows to assert generic constraints (encoded as parameters), e.g. MinimumAgeForDriving isConstraintFor John (where John is a legal subject under the TrafficLaw).
    The intended semantics (not expressible in OWL) is that a Parameter isConstraintFor and Entity if the Parameter isParameterFor a Concept that classifies that Entity; moreover, it entails that a Parameter parametrizes a Region that isRegionFor that Entity. The use in OWL is therefore a shortcut to annotate what Parameter constrains what Entity
    """
    
class isMemberOf(BaseProperty):
    """
    A relation between collections and entities, e.g. 'the Night Watch by Rembrandt is in the Rijksmuseum collection'; 'Davide is member of the Pen Club', 'Igor is one the subjects chosen for the experiment'.
    """
    
class includesWhole(BaseProperty):
    ...
    
class includesPart(BaseProperty):
    ...
    
class isPostconditionOf(BaseProperty):
    """
    Direct succession applied to situations. 
    E.g., 'Taking some rest is a postcondition of my search for a hotel'.
    """
    
class isPreconditionOf(BaseProperty):
    """
    Direct precedence applied to situations. 
    E.g., 'claiming to find nuclear weapons in a foreign country is a precondition to declare war against it'.
    """
    
class isPropertPartOf(BaseProperty):
    """
    http://www.ontologydesignpatterns.org/ont/dul/DUL.owl
    """
    
class hasTimeInterval(BaseProperty):
    """
    The generic relation between events and time intervals.
    """
    
class isTimeIntervalOf(BaseProperty):
    """
    The generic relation between time intervals and events.
    """
    
class includesAction(BaseProperty):
    """
    A relation between situations and actions, e.g. 'this morning I've prepared my coffee and had my fingers burnt' (i.e.: the preparation of my coffee this morning included a burning of my fingers).
    """
    
class isActionIncludedIn(BaseProperty):
    ...
    
class includesAgent(BaseProperty):
    """
    A relation between situations and persons, e.g. 'this morning I've prepared my coffee and had my fingers burnt' (i.e.: the preparation of my coffee this morning included me).
    """
    
class isAgentIncludedIn(BaseProperty):
    ...
    
class includesTime(BaseProperty):
    """
    A relation between situations and time intervals, e.g. 'this morning I've prepared my coffee and had my fingers burnt' (i.e.: preparing my coffee was held this morning). A data value attached to the time interval typically complements this modelling pattern.
    """
    
class isTimeIncludedIn(BaseProperty):
    ...
    
class introduces(BaseProperty):
    """
    A relation between a Description and a SocialAgent, e.g. a Constitutional Charter introduces the SocialAgent 'PresidentOfRepublic'.
    """
    
class isIntroducedBy(BaseProperty):
    """
    A relation between a Description and a SocialAgent, e.g. a Constitutional Charter introduces the SocialAgent 'PresidentOfRepublic'.
    """
    
class isAgentInvolvedIn(BaseProperty):
    """
    Agent participation.
    """
    
class isConceptUsedIn(BaseProperty):
    """
    A more generic relation holding between a Description and a Concept. In order to be used, a Concept must be previously definedIn another Description
    """
    
class isObservableAt(BaseProperty):
    """
    A relation to represent a (past, present or future) TimeInterval at which an Entity is observable.
    In order to encode a specific time, a data value should be related to the TimeInterval. 
    An alternative way of representing time is the datatype property: hasIntervalDate
    """
    
class isTimeOfObservationOf(BaseProperty):
    """
    A relation to represent a (past, present or future) TimeInterval at which an Entity is observable.
    In order to encode a specific time, a data value should be related to the TimeInterval. 
    An alternative way of representing time is the datatype property: hasIntervalDate
    """
    
class isParametrizedBy(BaseProperty):
    """
    The relation between a Parameter, e.g. 'MajorAge', and a Region, e.g. '>17 year'.
    """
    
class parametrizes(BaseProperty):
    """
    The relation between a Parameter, e.g. 'MajorAgeLimit', and a Region, e.g. '18_year'.
    For a more data-oriented relation, see hasDataValue
    """
    
class isReferenceOfInformationRealizedBy(BaseProperty):
    """
    The relation between entities and information realizations, e.g. between Italy and a paper copy of the text of the Italian Constitution.
    """
    
class realizesInformationAbout(BaseProperty):
    """
    The relation between entities and information realizations, e.g. between Italy and a paper copy of the text of the Italian Constitution.
    """
    
class isSatisfiedBy(BaseProperty):
    """
    A relation between a Situation and a Description, e.g. the execution of a Plan satisfies that plan.
    """
    
class isSpecializedBy(BaseProperty):
    """
    A partial order relation that holds between social objects. It represents the subsumption relation between e.g. a Concept and another Concept that is broader in extensional interpretation, but narrowe in intensional interpretation.
    E.g. PhDStudent Role specializes Student Role
    """
    
class specializes(BaseProperty):
    """
    A partial order relation that holds between social objects. 
    It mainly represents the subsumption relation between e.g. a Concept or Description and another Concept (resp. Description) that is broader in extensional interpretation, but narrower in intensional interpretation. For example, the role PhDStudent specializes the role Student.
    Another possible use is between a Collection that isCoveredBy a Concept A, and another Collection that isCoveredBy a Concept B that on its turm specializes A. For example, the 70,000 series Selmer Mark VI saxophone Collection specializes the Selmer Mark VI saxophone Collection.
    """
    
class isSubordinatedTo(BaseProperty):
    """
    Direct succession applied to concepts. E.g. the role 'Officer' is subordinated to 'Director'.
    """
    
class isSuperordinatedTo(BaseProperty):
    """
    Direct precedence applied to concepts. E.g. the role 'Executive' is superordinated to 'DepartmentManager'.
    """
    
class isUnifiedBy(BaseProperty):
    """
    A Collection has a unification criterion, provided by a Description; for example, a community of practice can be unified by a shared theory or interest, e.g. the community that makes research on mirror neurons shares some core knowledge about mirror neurons, which can be represented as a Description MirrorNeuronTheory that unifies the community. There can be several unifying descriptions.
    """
    
class unifies(BaseProperty):
    """
    A Collection has a unification criterion, provided by a Description; for example, a community of practice can be unified by a shared theory or interest, e.g. the community that makes research on mirror neurons shares some core knowledge about mirror neurons, which can be represented as a Description MirrorNeuronTheory that unifies the community. There can be several unifying descriptions.
    """
    
class nearTo(BaseProperty):
    """
    Generic distance relation between any Entity(s). E.g. Rome is near to Florence, astronomy is near to physics.
    """
    
class sameSettingAs(BaseProperty):
    """
    A relation between two entities participating in a same Situation; e.g., 'Our company provides an antivenom service' (the situation is the service, the two entities are the company and the antivenom).
    """
    
