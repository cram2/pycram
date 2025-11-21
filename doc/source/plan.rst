===============
The PyCRAM Plan
===============

.. _plan_header:

.. contents::
   :local:
   :depth: 1

What is a Plan?
===============
A ``Plan`` is the central coordination structure in PyCRAM. It captures "what to do next" for a robot by arranging
high‑level task descriptions (Designators), control‑flow (the Language layer), and the current execution context
(world and robot) into a single, navigable structure.

Think of a Plan as a living, directed tree of steps. Each step is a node (a ``PlanNode``). Some nodes represent
control‑flow (e.g., do A then B, do A and B in parallel, repeat, monitor). Other nodes represent concrete, executable
robot activities (e.g., a grasp action or a base motion). Executing a Plan means traversing this tree according to the
control‑flow nodes and carrying out the actions/motions at the leaves.

Why does the Plan matter?
=========================
The Plan is the backbone that holds PyCRAM together at run time:

- Integration point: It is where symbolic task specifications (Designators), procedural control‑flow (Language), and
  low‑level execution (Actions and Motions) meet.
- Single source of truth: It stores the current execution state (which step is running, succeeded, failed, or paused),
  making introspection, debugging, and visualization straightforward.
- Context carrier: It transports robot and world context through the whole execution, so every step can act with the
  correct scene and embodiment.
- Composability: Plans can be nested and mounted, letting you build complex behaviors from reusable sub‑plans.
- Observability and tooling: The Plan supports callbacks, monitoring, and visualization, enabling logging, dashboards,
  and supervision without changing task logic.

How a Plan is shaped (high level)
=================================

A Plan is usually a tree with a control‑flow ("language") node at the root and action/motion nodes beneath:

.. code-block:: text

   Plan
   └─ LanguageNode (e.g., Sequential, Parallel, Try, Repeat, Monitor, Code)
      ├─ Action/Motion Nodes (resolved or to be resolved from Designators)
      ├─ Mounted Sub‑Plans
      └─ More LanguageNodes (to structure the subtree)

- LanguageNodes define the order and concurrency of execution.
- Action/Motion nodes actually perform work on the robot based on Designators (see below).

PlanNodes in one sentence
=========================
A ``PlanNode`` is a step in the Plan. It knows its status (created/running/succeeded/failed/paused), its position in the
Plan (parent/children), and—if it is executable—how to perform its part.

How Designators connect to the Plan (conceptually)
==================================================
Designators are structured task descriptions ("pick up this object", "move base to that pose"). The Plan turns these into
executable nodes:

- Unresolved Action Designators become Action nodes that can be resolved at run time (late binding to a concrete
  action instance when more information is available).
- Resolved Action Designators become concrete Action nodes that execute immediately.
- Motion Designators (e.g., base/arm motions) become Motion nodes and execute directly.

This bridge lets high‑level reasoning produce Designators while the Plan ensures they are executed in the right order and
under the right conditions.

Where the Language fits
=======================
The Language layer provides the control‑flow vocabulary. In Plans, LanguageNodes such as ``Sequential``, ``Parallel``,
``TryInOrder``, ``TryAll``, ``Repeat``, and ``Monitor`` shape how and when children run. They do not manipulate robot
hardware themselves—they orchestrate the subtree beneath them.

Role in the overall framework
=============================

- Execution orchestration: Plans are the runtime engine that sequences, parallelizes, retries, and monitors robot
  activities.
- Context propagation: Every node sees the same world/robot context carried by the Plan, ensuring consistent decision
  making and execution.
- Abstraction boundary: The Plan is the stable interface between task specification (Designators and Language) and
  concrete execution (Actions/Motions, controllers, and external interfaces).
- Introspection and visualization: Plans expose structure and status for plotting, logging, and debugging, which is
  essential for real robot deployments.
- Composable building blocks: Larger behaviors are built by composing Plans (mounting sub‑plans) and mixing language
  constructs with designator‑driven actions.

Mental model
============

1. You describe what should happen using Designators and Language constructs.
2. The Plan assembles these descriptions into a tree of steps (nodes) bound to the current robot and world.
3. Executing the Plan walks the tree, letting LanguageNodes orchestrate and Action/Motion nodes act.
4. Throughout execution, the Plan tracks state, supports monitoring/callbacks, and makes it easy to inspect or adapt.

Practical takeaway
==================
Use Plans to turn high‑level intentions into reliable robot behavior: the Plan is your central, observable, and
composable execution graph that unifies task descriptions, control‑flow, and robot/world context.