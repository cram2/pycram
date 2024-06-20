from pycram.designators.action_designator import PickUpAction, PickUpActionPerformable
from pycram.local_transformer import LocalTransformer


class DualArmPickupAction(PickUpAction):
    """
    Specialization version of the PickUpAction designator which uses heuristics to solve for a dual grasping solution.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def ground(self) -> PickUpActionPerformable:
        pass
