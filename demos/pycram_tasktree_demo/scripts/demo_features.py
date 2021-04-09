import sys
import os
sys.path.append(os.getcwd() + "/../../../pycram/src/")
import inspect

import pycram
from pycram.task import with_tree, TaskTreeNode, Code, move_node_to_path

file_format = "pdf"

@with_tree
def top_level_plan():
    print("Top-level Plan: Executed.")
    plan_a("foo")
    plan_b()

@with_tree
def plan_a(param):
    print("Plan A: Executed with param: {}.".format(param))

@with_tree
def plan_b():
    print("Plan B: Executed.")
    plan_c()
    print("Plan B: Log between plan calls.")
    plan_c()

@with_tree
def plan_c():
    print("Plan C: Executed.")

@with_tree
def plan_d():
    print("Plan D: Executed.")

def insert_demo():
    print("="*10 + " Insert Demo " + "="*10)
    top_level_plan()
    node_b = TaskTreeNode(Code("", plan_b)) # Should be plan_b.__wrapped__ but this dosent work for some reason
    node_d = TaskTreeNode(Code("", plan_d)) # Should be plan_d.__wrapped__ but this dosent work for some reason
    pycram.task.TASK_TREE.get_child_by_path("plan_a").insert_after(node_d)
    pycram.task.TASK_TREE.generate_dot().render("demos/insert/1", format=file_format, view=False)
    pycram.task.TASK_TREE.execute()
    pycram.task.TASK_TREE.generate_dot().render("demos/insert/2", format=file_format, view=False)

    pycram.task.TASK_TREE.get_child_by_path("plan_a").insert_before(node_b)
    pycram.task.TASK_TREE.generate_dot().render("demos/insert/3", format=file_format, view=False)
    pycram.task.TASK_TREE.execute()
    pycram.task.TASK_TREE.generate_dot().render("demos/insert/4", format=file_format, view=False)

    pycram.task.TASK_TREE.get_child_by_path("plan_a").delete()
    pycram.task.TASK_TREE.execute()
    pycram.task.TASK_TREE.generate_dot().render("demos/delete/after_insert", format=file_format, view=False)

    ## Not really part of the demo, but I shouldn't forget about this:
    top_level_plan()
    try:
        pycram.task.TASK_TREE.insert_after(node_d)
    except AttributeError:
        pass # Inserting at root is not supported. (Doesn't make sense really.)

def delete_demo():
    print("="*10 + " Delete Demo " + "="*10)
    top_level_plan()
    pycram.task.TASK_TREE.get_child_by_path("plan_b/plan_c.1").delete()
    pycram.task.TASK_TREE.execute()
    pycram.task.TASK_TREE.generate_dot().render("demos/delete/1", format=file_format, view=False)
    pycram.task.TASK_TREE.get_child_by_path("plan_b").delete()
    pycram.task.TASK_TREE.execute()
    pycram.task.TASK_TREE.generate_dot().render("demos/delete/2", format=file_format, view=False)

def move_demo():
    print("="*10 + " Move Demo " + "="*10)
    top_level_plan()
    node = pycram.task.TASK_TREE.get_child_by_path("plan_b/plan_c")
    move_node_to_path(node, "plan_a", 1)
    pycram.task.TASK_TREE.execute()
    pycram.task.TASK_TREE.generate_dot().render("demos/move/1", format=file_format, view=False)
    top_level_plan()
    node = pycram.task.TASK_TREE.get_child_by_path("plan_a")
    move_node_to_path(node, "plan_b/plan_c", 0)
    pycram.task.TASK_TREE.execute()
    pycram.task.TASK_TREE.generate_dot().render("demos/move/2", format=file_format, view=False)

def change_params_demo():
    print("="*10 + " Change Params Demo " + "="*10)
    top_level_plan()
    pycram.task.TASK_TREE.get_child_by_path("plan_a").code.args = ("bar",)
    pycram.task.TASK_TREE.execute()
    pycram.task.TASK_TREE.generate_dot().render("demos/change_params/1", format=file_format, view=False)

top_level_plan()
pycram.task.TASK_TREE.generate_dot().render("demos/test_plan", format=file_format)
insert_demo()
#delete_demo()
#move_demo()
#change_params_demo()
