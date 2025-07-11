import os
import tempfile
from pathlib import Path

from pycram.testing import BulletWorldTestCase
from pycram.object_descriptors.urdf import ObjectDescription as URDFObject
from pycram.config import world_conf as conf


class TestCacheManager(BulletWorldTestCase):

    def test_generate_description_and_write_to_cache(self):
        cache_manager = self.world.cache_manager
        path = os.path.join(self.world.conf.resources_path, "worlds/apartment.urdf")
        extension = Path(path).suffix
        cache_path = os.path.join(self.world.conf.cache_dir, "apartment.urdf")
        apartment = URDFObject(path)
        apartment.generate_description_from_file(path, "apartment", extension, cache_path)
        self.assertTrue(cache_manager.is_cached(path, apartment))

    def test_update_cache_dir_with_relative_path(self):
        """
        Test that update_cache_dir_with_object correctly handles relative paths
        by searching in the data directories.
        """
        cache_manager = self.world.cache_manager
        
        # Use a relative path to a self-contained URDF in the resources directory
        relative_path = "objects/table.urdf"
        
        # Create URDF object description
        urdf_description = URDFObject()
        
        # Test the cache update with relative path
        cache_path = cache_manager.update_cache_dir_with_object(
            relative_path, 
            ignore_cached_files=True,
            object_description=urdf_description,
            object_name="table"
        )
        
        # Verify that the file was found and cached
        self.assertTrue(cache_manager.is_cached(relative_path, urdf_description))
        self.assertTrue(os.path.exists(cache_path))
        
        # Verify that the original_path was set correctly (should be absolute path found in data dir)
        self.assertIsNotNone(urdf_description.original_path)
        original_path = str(urdf_description.original_path)
        self.assertTrue(os.path.isabs(original_path))
        self.assertTrue(original_path.endswith("table.urdf"))

    def test_update_cache_dir_with_absolute_path(self):
        """
        Test that update_cache_dir_with_object correctly handles absolute paths
        by loading the file directly without searching in data directories.
        """
        cache_manager = self.world.cache_manager
        
        # Create a temporary file with a URDF content for testing
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as temp_file:
            temp_file.write("""<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>
</robot>""")
            temp_file_path = temp_file.name
        
        try:
            # Create URDF object description
            urdf_description = URDFObject()
            
            # Test the cache update with absolute path
            cache_path = cache_manager.update_cache_dir_with_object(
                temp_file_path, 
                ignore_cached_files=True,
                object_description=urdf_description,
                object_name="test_robot"
            )
            
            # Verify that the file was cached
            self.assertTrue(cache_manager.is_cached(temp_file_path, urdf_description))
            self.assertTrue(os.path.exists(cache_path))
            
            # Verify that the original_path was set to the absolute path directly
            self.assertEqual(urdf_description.original_path, temp_file_path)
            
        finally:
            # Clean up the temporary file
            os.unlink(temp_file_path)

    def test_update_cache_dir_with_nonexistent_absolute_path(self):
        """
        Test that update_cache_dir_with_object raises FileNotFoundError
        when an absolute path points to a non-existent file.
        """
        cache_manager = self.world.cache_manager
        
        # Create a non-existent absolute path
        nonexistent_path = "/tmp/nonexistent_robot.urdf"
        
        # Create URDF object description
        urdf_description = URDFObject()
        
        # Test that FileNotFoundError is raised
        with self.assertRaises(FileNotFoundError):
            cache_manager.update_cache_dir_with_object(
                nonexistent_path, 
                ignore_cached_files=True,
                object_description=urdf_description,
                object_name="nonexistent_robot"
            )

    def test_update_cache_dir_with_nonexistent_relative_path(self):
        """
        Test that update_cache_dir_with_object raises FileNotFoundError
        when a relative path cannot be found in data directories.
        """
        cache_manager = self.world.cache_manager
        
        # Create a non-existent relative path
        nonexistent_relative_path = "nonexistent_robot.urdf"
        
        # Create URDF object description
        urdf_description = URDFObject()
        
        # Test that FileNotFoundError is raised
        with self.assertRaises(FileNotFoundError):
            cache_manager.update_cache_dir_with_object(
                nonexistent_relative_path, 
                ignore_cached_files=True,
                object_description=urdf_description,
                object_name="nonexistent_robot"
            )
