# ORM

When adding parameters to the Actions/Motions, you need to add those in the ORM classes as well.
Make sure to set "init=False" and "default=default_value" in the mapped_column if the added parameter is not a required
parameter.

Example:

```python
class Action():

    new_parameter: float = 0.0

class ActionORM(Base):

    new_parameter: Mapped[float] = mapped_column(init=False, default=0.0)
```

When modifying an action by adding another action/motion as part of it make sure to modify the position_results assertion
in test_node in the test_orm.py and test_simulated_tree, test_tree_creation in test_task_tree.py accordingly. A helpful tecnique is to print the task_tree and count the poses,
so each action has at least one pose since it has a robot_state unlike motions, then you add the poses that are needed
in that action:
```python
root = pycram.tasktree.task_tree.root
print(RenderTree(root, style=AsciiStyle()))
```

Or you could just print the len(position_results) and use that number to modify the assertion.