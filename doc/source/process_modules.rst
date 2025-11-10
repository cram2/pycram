Process Modules in PyCRAM
=========================

Process modules are PyCRAM’s bridge from symbolic intentions to concrete robot motions. They encapsulate the low‑level
execution of motion designators and perception tasks, providing a stable, high‑level interface to both simulated and real
robots. By keeping the "how" of actuation and sensing behind a common abstraction, process modules let planners and plans
remain robot‑agnostic and execution‑aware without being robot‑specific.

What Process Modules Are
------------------------
A process module is a small object responsible for executing a specific kind of motion or skill. It exposes an ``execute``
method that receives a motion designator—such as moving the base or orienting the head—and performs the requested action.
The base class ``pycram.process_module.ProcessModule`` provides per‑module locking and an optional execution delay to make
simulated behavior more realistic. Concrete implementations override the protected ``_execute`` method, which is invoked
within a lock to serialize concurrent requests of the same type.

Internally, the base class supports an ``execution_delay`` that, when set, sleeps for the configured duration after each
execution. This is primarily used in simulation to avoid unrealistically instantaneous actions and to better reflect the
pacing of real systems.

Process Modules in the Architecture
-----------------------------------
In the overall architecture, plans and motion designators describe what should be achieved, while process modules translate
those descriptions into how they are executed on a given robot and in a given execution context. The selection and provisioning
of concrete modules is delegated to ``pycram.process_module.ProcessModuleManager`` and robot‑specific managers derived from
``ManagerBase``. Each manager supplies modules for categories such as navigation, looking, detecting, TCP motion, arm‑joint
motion, generic joint motion, gripper control, and opening/closing mechanisms.

Choosing Between Simulated and Real Execution
---------------------------------------------
PyCRAM switches execution context using the ``ExecutionType`` enum and light‑weight context helpers. Within a
``pycram.process_module.simulated_robot`` context (or when decorating a function with ``@with_simulated_robot``), the
manager returns simulated implementations. Within a ``pycram.process_module.real_robot`` context (or ``@with_real_robot``),
it returns real‑robot implementations. Entering the real‑robot context temporarily disables the artificial ``execution_delay``
so that commands are dispatched without additional sleeps; leaving the context restores the previous setting.

Default vs. Robot‑Specific Managers
-----------------------------------
Process modules are grouped and provided by a manager. The default manager (``DefaultManager`` in
``pycram.process_modules.default_process_modules``) registers itself under the robot name ``"default"`` and implements a
complete set of modules for both simulation and real execution. Robot‑specific managers can be introduced by subclassing
``ManagerBase`` and registering the instance with ``ProcessModuleManager``. At runtime the active robot is looked up by
name; if no specific manager is available, the system falls back to the default manager and emits a warning. This pattern
lets projects begin with broadly useful defaults—especially in simulation—and then override only the pieces that need
robot‑specific behavior or hardware drivers.

Example: Moving the Head (Default vs. Real)
-------------------------------------------
Head movement illustrates how simulation and real execution share the same intent but rely on different effectors.
In simulation, ``DefaultMoveHead`` computes the pan and tilt angles required to look at a target point. It transforms
the target into the local frames of the pan and tilt links, updates the simulated joint states directly in the world
model, and then notifies the world that the state changed. On a real robot, ``DefaultMoveHeadReal`` performs the same
geometric computations but delegates the actual motion to the control back‑end (``giskard``). Instead of writing states,
it sends a joint‑goal command to move the actuators, optionally enabling safe self‑collision policies along the way.
Plans continue to issue a ``LookingMotion`` designator, and the current execution context determines which implementation is used.

How Managers Provide Modules
----------------------------
Each manager overrides factory methods such as ``navigate()``, ``looking()``, ``detecting()``, ``move_tcp()``,
``move_arm_joints()``, ``move_joints()``, ``move_gripper()``, ``open()``, ``close()``, and ``move_tcp_waypoints()``.
The default manager inspects ``ProcessModuleManager.execution_type`` and returns the corresponding simulated or real
implementation. Managers also maintain one lock per module category to serialize concurrent calls of the same type,
mirroring typical hardware limitations—for example, moving a head is treated as a single‑threaded capability.

Obtaining a Manager from Plans
------------------------------
Motion designators delegate to the manager at execution time. For instance,
``robot_plans.motions.robot_body.MoveJointsMotion.perform`` retrieves a manager with
``ProcessModuleManager().get_manager(self.robot_view)`` and then calls the appropriate module’s ``execute`` method.
This keeps designators stateless and allows the manager to inject the right execution behavior based on the current
robot and context without changing plan code.

Simulated vs. Semi‑Real vs. Real
--------------------------------
Beyond purely simulated and purely real contexts, PyCRAM provides a ``SemiRealRobot`` context for hybrid scenarios.
While the default modules showcased here primarily distinguish between simulation and real execution, the semi‑real
mode enables mixed pipelines—for example, simulated perception combined with real motion—once corresponding modules
and selection logic are added to a manager.

Extending Process Modules
-------------------------
To add a new robot or override a specific behavior, create a subclass of ``ManagerBase`` for that robot’s name and
implement the relevant factory methods. Register the manager by importing it from ``pycram.process_modules`` so that
it is discovered. New process modules should subclass ``ProcessModule``, implement ``_execute`` with the required logic,
and use world transforms and robot semantics from the ``AbstractRobot`` and ``World`` APIs whenever possible. For hardware
back‑ends, prefer delegating to established interfaces such as ``giskard`` or action servers.

Key takeaways
-------------
- Process modules turn high‑level designators into concrete actions in a selected execution context.
- Managers select simulated or real implementations based on the active context and the current robot.
- Defaults work out of the box; robot‑specific managers can override only what they need.
- The head‑movement modules demonstrate shared geometry with distinct execution back‑ends for simulation and real robots.
- Locks and optional delays model real constraints and pacing without burdening plan logic.
