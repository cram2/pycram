:py:mod:`pycram.cache_manager`
==============================

.. py:module:: pycram.cache_manager


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.cache_manager.CacheManager




.. py:class:: CacheManager(cache_dir: str, data_directory: typing_extensions.List[str])


   The CacheManager is responsible for caching object description files and managing the cache directory.

   Initializes the CacheManager.

   :param cache_dir: The directory where the cached files are stored.
   :param data_directory: The directory where all resource files are stored.

   .. py:attribute:: mesh_extensions
      :type: typing_extensions.List[str]
      :value: ['.obj', '.stl']

      The file extensions of mesh files.

   .. py:method:: update_cache_dir_with_object(path: str, ignore_cached_files: bool, object_description: pycram.description.ObjectDescription, object_name: str) -> str

      Checks if the file is already in the cache directory, if not it will be preprocessed and saved in the cache.

      :param path: The path of the file to preprocess and save in the cache directory.
      :param ignore_cached_files: If True, the file will be preprocessed and saved in the cache directory even if it
       is already cached.
      :param object_description: The object description of the file.
      :param object_name: The name of the object.


   .. py:method:: generate_description_and_write_to_cache(path: str, name: str, extension: str, cache_path: str, object_description: pycram.description.ObjectDescription) -> None

      Generates the description from the file at the given path and writes it to the cache directory.

      :param path: The path of the file to preprocess.
      :param name: The name of the object.
      :param extension: The file extension of the file to preprocess.
      :param cache_path: The path of the file in the cache directory.
      :param object_description: The object description of the file.


   .. py:method:: write_to_cache(description_string: str, cache_path: str) -> None
      :staticmethod:

      Writes the description string to the cache directory.

      :param description_string: The description string to write to the cache directory.
      :param cache_path: The path of the file in the cache directory.


   .. py:method:: look_for_file_in_data_dir(path_object: pathlib.Path) -> str

      Looks for a file in the data directory of the World. If the file is not found in the data directory, this method
      raises a FileNotFoundError.

      :param path_object: The pathlib object of the file to look for.


   .. py:method:: create_cache_dir_if_not_exists()

      Creates the cache directory if it does not exist.


   .. py:method:: is_cached(path: str, object_description: pycram.description.ObjectDescription) -> bool

      Checks if the file in the given path is already cached or if
      there is already a cached file with the given name, this is the case if a .stl, .obj file or a description from
      the parameter server is used.

      :param path: The path of the file to check.
      :param object_description: The object description of the file.
      :return: True if there already exists a cached file, False in any other case.


   .. py:method:: check_with_extension(path: str) -> bool

      Checks if the file in the given ath exists in the cache directory including file extension.

      :param path: The path of the file to check.


   .. py:method:: check_without_extension(path: str, object_description: pycram.description.ObjectDescription) -> bool

      Checks if the file in the given path exists in the cache directory without file extension,
      the extension is added after the file name manually in this case.

      :param path: The path of the file to check.
      :param object_description: The object description of the file.



