from ...datastructures.decision_tree import Decision, Query
from anytree import RenderTree

object_tree = None

context = (Decision(lambda designator: designator.type == "context")
           + Decision(lambda designator: designator.type == "object")
           - Query(lambda designator: object_tree.query_grasp_for_object(designator)))

print(RenderTree(context))


