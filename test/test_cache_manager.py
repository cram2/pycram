from pathlib import Path

from bullet_world_testcase import BulletWorldTestCase
from pycram.datastructures.enums import ObjectType
from pycram.object_descriptors.urdf import ObjectDescription as URDFObject
import pathlib


class TestCacheManager(BulletWorldTestCase):

    def test_generate_description_and_write_to_cache(self):
        cache_manager = self.world.cache_manager
        file_path = pathlib.Path(__file__).parent.resolve()
        path = str(file_path) + "/../resources/apartment.urdf"
        extension = Path(path).suffix
        cache_path = self.world.cache_dir + "/apartment.urdf"
        apartment = URDFObject(path)
        cache_manager.generate_description_and_write_to_cache(path, "apartment", extension, cache_path,
                                                              apartment)
        self.assertTrue(cache_manager.is_cached(path, apartment))
