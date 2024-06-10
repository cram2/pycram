from ....designators.location_designator import CostmapLocation


class DualGraspingLocation(CostmapLocation):
    """
    Specialization version of the CostmapLocation which uses heuristics to solve for a dual grasping solution.
    """

    def __iter__(self) -> CostmapLocation.Location:
        pass
    