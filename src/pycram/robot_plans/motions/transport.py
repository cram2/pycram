from dataclasses import dataclass

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.world_concepts.world_object import Object


@dataclass
class TransportingMotion:
    object_: Object
    arm: Arms
    target_location: PoseStamped

    def perform(self):
        # Grasp the object (geometry + planning encapsulated)
        self.plan_grasp()

        # Move to target location
        self.plan_transport()

        # Place the object
        self.plan_place()

        print(f"Transport complete: {self.object_.name} → {self.target_location}")
        return True

    def plan_grasp(self):
        # Generate grasp pose, call motion primitives
        print(f"Planning grasp of {self.object_.name} with {self.arm}")
        # Example: MoveTCPMotion(grasp_pose, self.arm).perform()

    def plan_transport(self):
        # Plan safe path to target
        print(f"Planning transport path to {self.target_location}")

    def plan_place(self):
        # Place object at target
        print(f"Planning placement at {self.target_location}")
