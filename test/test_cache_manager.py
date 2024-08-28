import os
from pathlib import Path

from bullet_world_testcase import BulletWorldTestCase
from pycram.object_descriptors.urdf import ObjectDescription as URDFObject


class TestCacheManager(BulletWorldTestCase):

    def test_generate_description_and_write_to_cache(self):
        cache_manager = self.world.cache_manager
        path = os.path.join(self.world.resources_path, "objects/apartment.urdf")
        extension = Path(path).suffix
        cache_path = os.path.join(self.world.cache_dir, "apartment.urdf")
        apartment = URDFObject(path)
        cache_manager.generate_description_and_write_to_cache(path, "apartment", extension, cache_path,
                                                              apartment)
        self.assertTrue(cache_manager.is_cached(path, apartment))
