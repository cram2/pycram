PyCRAM ORM: An Overview
===============================

What the ORM Is
---------------
The Object–Relational Mapping (ORM) in PyCRAM is the layer that translates the framework’s in-memory datastructures
into a relational database representation. It provides a structured way to persist plans, actions, spatial information,
and execution outcomes so that they can be retrieved, analyzed, and compared across runs.

What It Is Used For
-------------------
The ORM is used to capture the structure and history of robot behavior. It stores plan hierarchies and their execution
traces, the action intentions and the resolved actions that occurred at runtime, the spatial poses and transforms relevant
to those actions, and the metadata that characterizes each execution. By turning these elements into queryable records,
the ORM supports experiment tracking, benchmarking, and post hoc analysis.

Why It Is Helpful
-----------------
The ORM makes complex behavior legible and reproducible. It allows practitioners to recover the exact sequence of
decisions and motions that took place, with timing and status information, and to relate those events to objects and
places in the world. It enables analysts to ask performance questions at scale, to identify failure modes, and to audit
how plan variants behave across different contexts. It also fosters interoperability by using a database format that is
compatible with standard tooling for visualization and analytics.

How the Model Is Generated
--------------------------
PyCRAM uses an automated generator to keep the ORM aligned with the evolving domain model. The generator discovers
relevant classes from the planning and action layers, from spatial data structures, and from semantic world entities,
and then emits a coherent relational schema with mapped Python classes. Custom type handling is included for commonly
used scientific data types such as numerical arrays, ensuring that rich data can be stored efficiently. This approach
reduces manual maintenance and keeps the schema consistent with the rest of the framework.

What the Model Contains
-----------------------
The generated model captures several categories of information:

- Planning structure and execution state: The model records nodes within plans, their relationships, their statuses over time, and their start and end timestamps.
- Action designators and realized actions: The model represents high-level action descriptions and links them to the actions that were ultimately carried out, including the parameters that guided those actions.
- Spatial and temporal context: The model stores poses, orientations, transforms, and stamped headers so that actions can be analyzed in relation to time and reference frames.
- World entities and semantics: The model links behavior to the semantic representations of bodies and objects in the environment in order to relate actions to real or simulated entities.
- Execution metadata: The model captures run-specific context such as initial and final world states and the relevant robot and object poses, enabling detailed comparisons between executions.

How It Fits into the Workflow
-----------------------------
During execution, PyCRAM creates instances of actions, plan nodes, and spatial constructs as part of normal planning and
ontrol. The ORM layer provides a uniform pathway for persisting these instances and their relationships to a database.
Using this ORM layer a user can simply insert the whole plan into a database, the ORM will take care of recursively
discovering all the relevant information from the plan and translating them into a format that can be inserted into the
database.
Afterward, users and tools can query the stored data to reproduce runs, examine performance, generate reports, or
visualize plan structures and outcomes.

Key Takeaways
-------------
- The ORM provides a robust bridge between PyCRAM’s robotics abstractions and relational databases, allowing complex behavior to be stored and queried as structured data.
- The schema is generated automatically to mirror the domain model and to include custom support for scientific data, which significantly reduces manual effort.
- The stored information covers plans, actions, spatial context, world semantics, and execution metadata, which together enable reproducibility, analytics, and audits.
- The ORM integrates naturally into the execution workflow, making it straightforward to capture and later analyze what happened and why.

Additional Links
----------------
:ref:`ORM example<orm_example>`

