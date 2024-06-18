from pycram.designators.action_designator import PickUpAction, PickUpActionPerformable


class DualGraspingAction(PickUpAction):
    """
    Specialization version of the PickUpAction designator which uses heuristics to solve for a dual grasping solution.
    """

    def __iter__(self) -> PickUpActionPerformable:
        pass
    