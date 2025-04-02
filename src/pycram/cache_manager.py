from __future__ import annotations

import glob
import os
import pathlib
import shutil

from typing_extensions import List, TYPE_CHECKING, Optional

from .ros import  loginfo

if TYPE_CHECKING:
    from .description import ObjectDescription
    from .datastructures.dataclasses import Color
    from .datastructures.pose import TransformStamped


class CacheManager:

    """
    The CacheManager is responsible for caching object description files and managing the cache directory.
    """

    cache_cleared: bool = False
    """
    Indicate whether the cache directory has been cleared at least once since beginning or not.
    """

    def __init__(self, cache_dir: str, data_directory: List[str], clear_cache: bool = True):
        """
        Initialize the CacheManager.

        :param cache_dir: The directory where the cached files are stored.
        :param data_directory: The directory where all resource files are stored.
        :param clear_cache: If True, the cache directory will be cleared.
        """
        self.cache_dir = cache_dir
        self.data_directories = data_directory
        if clear_cache:
            self.clear_cache()

    def clear_cache(self):
        """
        Clear the cache directory.
        """
        self.delete_cache_dir()
        self.create_cache_dir_if_not_exists()
        self.cache_cleared = True

    def delete_cache_dir(self):
        """
        Delete the cache directory.
        """
        if pathlib.Path(self.cache_dir).exists():
            shutil.rmtree(self.cache_dir)

    def update_cache_dir_with_object(self, path: str, ignore_cached_files: bool,
                                     object_description: ObjectDescription, object_name: str,
                                     scale_mesh: Optional[float] = None,
                                     mesh_transform: Optional[TransformStamped] = None,
                                     color: Optional[Color] = None) -> str:
        """
        Check if the file is already in the cache directory, if not preprocess and save in the cache.

        :param path: The path of the file to preprocess and save in the cache directory.
        :param ignore_cached_files: If True, the file will be preprocessed and saved in the cache directory even if it
         is already cached.
        :param object_description: The object description of the file.
        :param object_name: The name of the object.
        :param scale_mesh: The scale of the mesh.
        :param mesh_transform: The transformation matrix to apply to the mesh.
        :param color: The color of the object.
        :return: The path of the cached file.
        """
        path_object = pathlib.Path(path)
        extension = path_object.suffix

        self.create_cache_dir_if_not_exists()

        # save correct path in case the file is already in the cache directory
        cache_path = os.path.join(self.cache_dir, object_description.get_file_name(path_object, extension, object_name))

        if not self.is_cached(path, object_description) or ignore_cached_files:
            # if file is not yet cached preprocess the description file and save it in the cache directory.
            path = self.look_for_file_in_data_dir(path_object)
            object_description.original_path = path
            object_description.generate_description_from_file(path, object_name, extension, cache_path,
                                                              scale_mesh, mesh_transform, color)

        return cache_path

    def look_for_file_in_data_dir(self, path_object: pathlib.Path) -> str:
        """
        Look for a file in the data directory of the World. If the file is not found in the data directory, raise a
         FileNotFoundError.

        :param path_object: The pathlib object of the file to look for.
        """
        name = path_object.name
        for data_dir in self.data_directories:
            data_path = pathlib.Path(data_dir).joinpath("**")
            for file in glob.glob(str(data_path), recursive=True):
                file_path = pathlib.Path(file)
                if file_path.name == name:
                    loginfo(f"Found file {name} in {file_path}")
                    return str(file_path)

        raise FileNotFoundError(
            f"File {name} could not be found in the resource directory {self.data_directories}")

    def create_cache_dir_if_not_exists(self):
        """
        Create the cache directory if it does not exist.
        """
        if not pathlib.Path(self.cache_dir).exists():
            os.mkdir(self.cache_dir)

    def is_cached(self, path: str, object_description: ObjectDescription) -> bool:
        """
        Check if the file in the given path is already cached or if
        there is already a cached file with the given name, this is the case if a .stl, .obj file or a description from
        the parameter server is used.

        :param path: The path of the file to check.
        :param object_description: The object description of the file.
        :return: True if there already exists a cached file, False in any other case.
        """
        return self.check_with_extension(path) or self.check_without_extension(path, object_description)

    def check_with_extension(self, path: str) -> bool:
        """
        Check if the file in the given ath exists in the cache directory including file extension.

        :param path: The path of the file to check.
        """
        file_name = pathlib.Path(path).name
        full_path = pathlib.Path(os.path.join(self.cache_dir, file_name))
        return full_path.exists()

    def check_without_extension(self, path: str, object_description: ObjectDescription) -> bool:
        """
        Check if the file in the given path exists in the cache directory the given file extension.
        Instead, replace the given extension with the extension of the used ObjectDescription and check for that one.

        :param path: The path of the file to check.
        :param object_description: The object description of the file.
        """
        file_stem = pathlib.Path(path).stem
        full_path = pathlib.Path(os.path.join(self.cache_dir, file_stem + object_description.get_file_extension()))
        return full_path.exists()
