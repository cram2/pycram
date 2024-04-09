from pathlib import Path

from bullet_world_testcase import BulletWorldTestCase
from pycram.datastructures.enums import ObjectType
from pycram.world_concepts.world_object import Object
import pathlib


class TestCacheManager(BulletWorldTestCase):

    def test_generate_description_and_write_to_cache(self):
        cache_manager = self.world.cache_manager
        file_path = pathlib.Path(__file__).parent.resolve()
        path = str(file_path) + "/../resources/apartment.urdf"
        extension = Path(path).suffix
        cache_path = self.world.cache_dir + "apartment.urdf"
        apartment = Object("apartment", ObjectType.ENVIRONMENT, path)
        cache_manager.generate_description_and_write_to_cache(path, apartment.name, extension, cache_path,
                                                              apartment.description)
        self.assertTrue(cache_manager.is_cached(path, apartment.description))
