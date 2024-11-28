import os
from pathlib import Path

from pycram.testing import BulletWorldTestCase
from pycram.object_descriptors.urdf import ObjectDescription as URDFObject
from pycram.config import world_conf as conf


class TestCacheManager(BulletWorldTestCase):

    def test_generate_description_and_write_to_cache(self):
        cache_manager = self.world.cache_manager
        path = os.path.join(self.world.conf.resources_path, "objects/apartment.urdf")
        extension = Path(path).suffix
        cache_path = os.path.join(self.world.conf.cache_dir, "apartment.urdf")
        apartment = URDFObject(path)
        apartment.generate_description_from_file(path, "apartment", extension, cache_path)
        self.assertTrue(cache_manager.is_cached(path, apartment))
