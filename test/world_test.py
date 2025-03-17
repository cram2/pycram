import unittest

# Import the necessary modules from your project.
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.world_factory import WorldFactory
from pycram.datastructures.world import World

# Create a dummy concrete subclass of World that implements all abstract methods minimally.
class DummyWorld(World):
    def _init_world(self, mode):
        # No actual simulation needed for the test.
        pass

    def load_object_and_get_id(self, path=None, pose=None, obj_type=None):
        return 0

    def _remove_visual_object(self, obj_id: int) -> bool:
        return True

    def remove_object_from_simulator(self, obj):
        return True

    def _get_joint_position(self, joint):
        return 0.0

    def get_object_joint_names(self, obj):
        return []

    def get_link_pose(self, link):
        return None

    def get_multiple_link_poses(self, links):
        return {}

    def get_link_position(self, link):
        return [0, 0, 0]

    def get_link_orientation(self, link):
        return [0, 0, 0, 1]

    def get_multiple_link_positions(self, links):
        return {}

    def get_multiple_link_orientations(self, links):
        return {}

    def get_object_link_names(self, obj):
        return []

    def step(self, func=None, step_seconds=None):
        pass

    def get_object_pose(self, obj):
        return None

    def get_multiple_object_poses(self, objects):
        return {}

    def get_multiple_object_positions(self, objects):
        return {}

    def get_object_position(self, obj):
        return [0, 0, 0]

    def get_multiple_object_orientations(self, objects):
        return {}

    def get_object_orientation(self, obj):
        return [0, 0, 0, 1]

    def perform_collision_detection(self):
        pass

    def get_body_contact_points(self, body):
        return []

    def _reset_joint_position(self, joint, joint_position):
        return True

    def _set_multiple_joint_positions(self, joint_positions):
        return True

    def _get_multiple_joint_positions(self, joints):
        return {}

    def reset_object_base_pose(self, obj, pose):
        return True

    def reset_multiple_objects_base_poses(self, objects):
        return True

    def join_threads(self):
        pass

    def disconnect_from_physics_server(self):
        pass

    def save_physics_simulator_state(self, state_id=None, use_same_id=False):
        return 0

    def remove_physics_simulator_state(self, state_id: int):
        pass

    def restore_physics_simulator_state(self, state_id: int):
        pass

    def _ray_test(self, from_position, to_position):
        # For testing, return a dummy RayResult object.
        from pycram.datastructures.dataclasses import RayResult
        return RayResult()

    def _ray_test_batch(self, from_positions, to_positions, num_threads=1):
        return [self._ray_test(fp, tp) for fp, tp in zip(from_positions, to_positions)]

    def _create_visual_shape(self, visual_shape):
        return 1

    def _create_multi_body(self, multi_body):
        return 1

    def _create_box_visual_shape(self, shape_data):
        return 1

    def _create_cylinder_visual_shape(self, shape_data):
        return 1

    def _create_sphere_visual_shape(self, shape_data):
        return 1

    def _create_capsule_visual_shape(self, shape_data):
        return 1

    def _create_plane_visual_shape(self, shape_data):
        return 1

    def _create_mesh_visual_shape(self, shape_data):
        return 1

    def _add_text(self, text, position, orientation=None, size=0.1, color=None, life_time=0,
                  parent_object_id=None, parent_link_id=None):
        return 1

    def _remove_text(self, text_id=None):
        pass

# To test WorldFactory without changing your production code,
# monkey-patch the World class used by WorldFactory with our DummyWorld.
def patch_world_factory():
    import pycram.datastructures.world_factory as wf
    wf.World = DummyWorld

class TestWorldFactory(unittest.TestCase):
    def setUp(self):
        # Ensure the factory uses our dummy concrete world.
        patch_world_factory()
        # Clear any previously registered worlds.
        WorldFactory.clear_registry()

    def test_create_belief_world(self):
        # Create a belief world; this should become the main world.
        belief_world = WorldFactory.create_world("belief", mode=WorldMode.DIRECT, clear_cache=True)
        # Check that the world is not a prospection world.
        self.assertFalse(belief_world.is_prospection_world)
        # In our design, the belief world should be set as the global current world.
        self.assertEqual(belief_world, DummyWorld.current_world)

    def test_create_prospection_world(self):
        # Create a prospection world.
        prospection_world = WorldFactory.create_world("prospection", mode=WorldMode.DIRECT, clear_cache=False)
        # It should be flagged as a prospection world.
        self.assertTrue(prospection_world.is_prospection_world)
        # A prospection world should not override the main (belief) world.
        self.assertNotEqual(prospection_world, DummyWorld.current_world)

    def test_registry(self):
        # Create two worlds and check the registry.
        w1 = WorldFactory.create_world("belief", mode=WorldMode.DIRECT)
        w2 = WorldFactory.create_world("prospection", mode=WorldMode.DIRECT)
        worlds = WorldFactory.list_worlds()
        self.assertEqual(len(worlds), 2)
        self.assertIsNotNone(WorldFactory.get_world(0))
        self.assertIsNotNone(WorldFactory.get_world(1))

    def test_unregister_and_clear_registry(self):
        # Create a world, unregister it, and then clear the registry.
        w1 = WorldFactory.create_world("belief", mode=WorldMode.DIRECT)
        WorldFactory.unregister_world(w1.id)
        self.assertIsNone(WorldFactory.get_world(w1.id))
        WorldFactory.clear_registry()
        self.assertEqual(len(WorldFactory.list_worlds()), 0)

if __name__ == "__main__":
    unittest.main()
