import os
import unittest
from typing import Dict, List

# Import necessary modules from your project.
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.world_factory import WorldFactory
from pycram.datastructures.world import World

# Additional imports for dummy implementations.
from pycram.datastructures.dataclasses import (
    Color, ContactPointsList, MultiBody, VisualShape, BoxVisualShape,
    CylinderVisualShape, SphereVisualShape, CapsuleVisualShape, PlaneVisualShape,
    MeshVisualShape, RayResult
)
from pycram.datastructures.pose import Pose, Transform
from pycram.world_concepts.constraints import Constraint

# --- DummyLocalTransformer ---
class DummyLocalTransformer:
    def __init__(self, *args, **kwargs):
        pass

# --- DummyWorld ---
class DummyWorld(World):
    def __init__(self, mode=WorldMode.DIRECT, is_prospection=False, clear_cache=False, id_=-1, **kwargs):
        # Minimal initialization that bypasses ROS-related code.
        self.id = id_  # Set the world id.
        self.ontology = None
        self.is_prospection_world = is_prospection
        self.latest_state_id = 0
        self.objects = []
        self.coll_callbacks = {}
        self.on_add_object_callbacks = []
        # Use our dummy local transformer.
        self.local_transformer = DummyLocalTransformer()
        # For testing, set prospection_world to self and disable syncing.
        self.prospection_world = self
        self.world_sync = None
        if World.current_world is None:
            World.current_world = self
        self.original_state_id = 0
        # Use __dict__ to bypass read-only property restrictions.
        self.__dict__['saved_states'] = {0: None}
        self._init_world(mode)

    def _init_world(self, mode: WorldMode):
        pass

    # Override exit so that no cleanup is attempted.
    def exit(self, remove_saved_states: bool = True) -> None:
        pass

    def load_object_and_get_id(self, path=None, pose: Pose = None, obj_type=None):
        return 0

    def _remove_visual_object(self, obj_id: int) -> bool:
        return True

    def remove_object_from_simulator(self, obj):
        return True

    def _get_joint_position(self, joint) -> float:
        return 0.0

    def get_object_joint_names(self, obj) -> List[str]:
        return []

    def get_link_pose(self, link) -> Pose:
        return Pose()

    def get_multiple_link_poses(self, links: List) -> Dict[str, Pose]:
        return {}

    def get_link_position(self, link) -> List[float]:
        return [0, 0, 0]

    def get_link_orientation(self, link) -> List[float]:
        return [0, 0, 0, 1]

    def get_multiple_link_positions(self, links: List) -> Dict[str, List[float]]:
        return {}

    def get_multiple_link_orientations(self, links: List) -> Dict[str, List[float]]:
        return {}

    def get_object_link_names(self, obj) -> List[str]:
        return []

    def step(self, func=None, step_seconds: float = None) -> None:
        pass

    def get_object_pose(self, obj) -> Pose:
        return Pose()

    def get_multiple_object_poses(self, objects: List) -> Dict[str, Pose]:
        return {}

    def get_multiple_object_positions(self, objects: List) -> Dict[str, List[float]]:
        return {}

    def get_object_position(self, obj) -> List[float]:
        return [0, 0, 0]

    def get_multiple_object_orientations(self, objects: List) -> Dict[str, List[float]]:
        return {}

    def get_object_orientation(self, obj) -> List[float]:
        return [0, 0, 0, 1]

    def perform_collision_detection(self) -> None:
        pass

    def get_body_contact_points(self, body) -> ContactPointsList:
        return ContactPointsList([])

    def _reset_joint_position(self, joint, joint_position: float) -> bool:
        return True

    def _set_multiple_joint_positions(self, joint_positions: Dict) -> bool:
        return True

    def _get_multiple_joint_positions(self, joints: List) -> Dict[str, float]:
        return {}

    def reset_object_base_pose(self, obj, pose: Pose) -> bool:
        return True

    def reset_multiple_objects_base_poses(self, objects: Dict) -> bool:
        return True

    def join_threads(self) -> None:
        pass

    def disconnect_from_physics_server(self) -> None:
        pass

    def save_physics_simulator_state(self, state_id: int = None, use_same_id: bool = False) -> int:
        return 0

    def remove_physics_simulator_state(self, state_id: int) -> None:
        pass

    def restore_physics_simulator_state(self, state_id: int) -> None:
        pass

    def _ray_test(self, from_position: List[float], to_position: List[float]) -> RayResult:
        return RayResult()

    def _ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]], num_threads: int = 1) -> List[RayResult]:
        return [self._ray_test(fp, tp) for fp, tp in zip(from_positions, to_positions)]

    def _create_visual_shape(self, visual_shape: VisualShape) -> int:
        return 1

    def _create_multi_body(self, multi_body: MultiBody) -> int:
        return 1

    def _create_box_visual_shape(self, shape_data: BoxVisualShape) -> int:
        return 1

    def _create_cylinder_visual_shape(self, shape_data: CylinderVisualShape) -> int:
        return 1

    def _create_sphere_visual_shape(self, shape_data: SphereVisualShape) -> int:
        return 1

    def _create_capsule_visual_shape(self, shape_data: CapsuleVisualShape) -> int:
        return 1

    def _create_plane_visual_shape(self, shape_data: PlaneVisualShape) -> int:
        return 1

    def _create_mesh_visual_shape(self, shape_data: MeshVisualShape) -> int:
        return 1

    def _add_text(self, text: str, position: List[float], orientation: List[float] = None, size: float = 0.1,
                  color: Color = Color(1, 1, 1, 1), life_time: float = 0,
                  parent_object_id: int = None, parent_link_id: int = None) -> int:
        return 1

    def _remove_text(self, text_id: int = None) -> None:
        pass

    # Implement missing abstract methods:
    def add_constraint(self, constraint: Constraint) -> int:
        return 0

    def remove_constraint(self, constraint_id) -> None:
        pass

    def get_colors_of_object_links(self, obj) -> Dict[str, Color]:
        return {}

    def get_contact_points_between_two_bodies(self, body_1, body_2) -> ContactPointsList:
        return ContactPointsList([])

    def get_link_color(self, link) -> Color:
        return Color(1, 1, 1, 1)

    def set_gravity(self, gravity_vector: List[float]) -> None:
        pass

    def set_link_color(self, link, rgba_color: Color) -> None:
        pass

    def set_realtime(self, real_time: bool) -> None:
        pass

    def __del__(self):
        # Override __del__ to avoid cleanup errors.
        pass

# Monkey-patch World in WorldFactory with DummyWorld.
def patch_world_factory():
    import pycram.datastructures.world_factory as wf
    wf.World = DummyWorld

class TestWorldFactory(unittest.TestCase):
    def setUp(self):
        # Ensure the cache directory exists so clear_cache() does not fail.
        cache_dir = '/home/mohammad/workspace/ros/src/resources/cached'
        os.makedirs(cache_dir, exist_ok=True)
        # Patch LocalTransformer with DummyLocalTransformer.
        try:
            import pycram.local_transformer
            pycram.local_transformer.LocalTransformer = DummyLocalTransformer
        except ImportError:
            pass
        # Monkey-patch World in WorldFactory.
        patch_world_factory()
        WorldFactory.clear_registry()

    def test_create_belief_world(self):
        belief_world = WorldFactory.create_world("belief", mode=WorldMode.DIRECT, clear_cache=True)
        self.assertFalse(belief_world.is_prospection_world)
        self.assertEqual(belief_world, DummyWorld.current_world)

    def test_create_prospection_world(self):
        prospection_world = WorldFactory.create_world("prospection", mode=WorldMode.DIRECT, clear_cache=False)
        self.assertTrue(prospection_world.is_prospection_world)
        self.assertNotEqual(prospection_world, DummyWorld.current_world)

    def test_registry(self):
        w1 = WorldFactory.create_world("belief", mode=WorldMode.DIRECT)
        w2 = WorldFactory.create_world("prospection", mode=WorldMode.DIRECT)
        worlds = WorldFactory.list_worlds()
        self.assertEqual(len(worlds), 2)
        self.assertIsNotNone(WorldFactory.get_world(0))
        self.assertIsNotNone(WorldFactory.get_world(1))

    def test_unregister_and_clear_registry(self):
        w1 = WorldFactory.create_world("belief", mode=WorldMode.DIRECT)
        WorldFactory.unregister_world(w1.id)
        self.assertIsNone(WorldFactory.get_world(w1.id))
        WorldFactory.clear_registry()
        self.assertEqual(len(WorldFactory.list_worlds()), 0)

if __name__ == "__main__":
    unittest.main()
