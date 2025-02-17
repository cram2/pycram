import fnmatch
import os

import requests
from bs4 import BeautifulSoup
from ..ros import  ros_tools
from typing_extensions import Dict, Set
from ..ros import  logging as log

filename = ros_tools.get_ros_package_path('pycram')


class ProcThorInterface:
    def __init__(self, base_url="http://procthor.informatik.uni-bremen.de:5000",
                 source_folder=os.path.join(ros_tools.get_ros_package_path('pycram'), "resources/tmp/")):
        self.base_url = base_url
        self.source_folder = source_folder
        self.stored_environments = []
        self.stored_environments.extend(self.get_all_environments_stored_below_directory(self.source_folder))

    def _download_file(self, base_url: str, full_url: str, folder: str) -> str:
        """
        Downloads the file given in full_url and stores it into folder. If necessary it will create the same folder
        structure. For this purpose the base_url is necessary to decide what is folder structure and what is just url.


        :param base_url: Base url as string from
        :param full_url: Full url of the file to be downloaded
        :param folder: Folder where the file should be stored
        :return: The local file name of the downloaded file
        """
        tree_structure = full_url.replace(base_url, '')
        if tree_structure.startswith("/"):
            tree_structure = tree_structure[1:]
        full_storage_path = os.path.join(folder, tree_structure)
        if not os.path.exists(full_storage_path[:full_storage_path.rfind('/') + 1]):
            os.makedirs(full_storage_path[:full_storage_path.rfind('/') + 1], exist_ok=True)
        local_filename = os.path.join(folder, full_url.split('/')[-1])
        # Send HTTP GET request to fetch the file
        with requests.get(full_url, stream=True) as r:
            r.raise_for_status()
            # Write the content to a file
            with open(full_storage_path, 'wb') as f:
                for chunk in r.iter_content(chunk_size=8192):
                    f.write(chunk)
        return local_filename

    # Function to get the list of files in a directory on the web server
    def _get_files_list(self, base_url: str) -> Set[str]:
        """
        Function to get the list of files in a directory on the web server.

        :param base_url: Base url as string from
        :return: Set of all files found below the given url
        """
        # Send GET request to the URL
        response = requests.get(base_url)
        # Parse the HTML response
        soup = BeautifulSoup(response.text, 'html.parser')
        # Find all links on the page
        links = soup.find_all('a')
        file_list = set()
        files_urls = set()
        for link in links:
            if link.get('href').endswith('/') and not link.get('href').startswith('/'):
                files_urls = files_urls.union(self._get_files_list(os.path.join(base_url, link['href'])))

            elif link.get('href').endswith(('.usda', '.urdf', '.stl', '.usd', '.hdr')):
                files_urls.add(os.path.join(base_url, link['href']))
            else:
                log.logwarn("Ignored File: {}".format(link.get('href')))
        return files_urls

    # Main function to download all files from a directory
    def download_all_files_from_URL(self, base_url: str, folder: str) -> None:
        """
        Main function to download all files from a given path. Files will be stored in folder.

        :param base_url: Base url as string from
        :param folder: folder where the files should be stored
        :return: None
        """
        # Ensure the folder exists
        os.makedirs(folder, exist_ok=True)
        # Get the list of files
        files = self._get_files_list(base_url)
        # create_folder_structure(base_url,folder,files)
        # Download each file
        for file_url in files:
            log.loginfo(f"Downloading {file_url}...")
            self._download_file(base_url, file_url, folder)
        log.loginfo("All files downloaded.")
        return None

    # Returns a list of all the urdf files in a given folder structure if non is given uses the base_source_folder
    # only look for urdf, but can be easly adapted
    def get_all_environments_stored_below_directory(self, source_folder: str) -> list:
        """
        Returns a list of dictionaries that contains the name and the storage_place of all urdf files below a given
        folder. Only looks for urdf.

        :param source_folder: Base path as string from
        :return: map from name to storage place
        """
        urdf_files = []
        urdf_file = {}
        for root, dirs, files in os.walk(source_folder):
            for filename in fnmatch.filter(files, '*.urdf'):
                urdf_file["name"] = filename.rsplit('.', 1)[0]
                urdf_file["storage_place"] = os.path.join(root, filename)
                urdf_files.append(urdf_file.copy())
        return urdf_files

    def download_one_random_environment(self) -> Dict[str, str]:
        """
        Downloads one random environment currently not present.

        :param: None
        :return: Dictionary of name and storage place
        """
        endpoint = "GetRandomEnvironment"
        full_url = os.path.join(self.base_url, endpoint)
        response = requests.get(full_url)
        self.download_all_files_from_URL(response.json()["storage_place"], self.source_folder)
        self.stored_environments.append(response.json())
        return response.json()

    def download_num_random_environment(self, num_of_environments: int) -> list:
        """
        Downloads given amount of  random environment currently not present.

        :param num_of_environments: Amount of environments that get downloaded
        :return: List of Dictionaries of name and storage place
        """
        endpoint = "GetNumEnvironment"
        full_url = os.path.join(self.base_url, endpoint, str(num_of_environments))
        response = requests.get(full_url)
        for env in response.json():
            self.download_all_files_from_URL(env["storage_place"], self.source_folder)
            self.stored_environments.append(env)
        return response.json()
