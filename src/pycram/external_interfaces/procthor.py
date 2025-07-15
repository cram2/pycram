import os
import random
import tarfile
from dataclasses import dataclass
from typing import List
from urllib.parse import urljoin

import requests
from bs4 import BeautifulSoup

from pycram.config.world_conf import WorldConfig


@dataclass
class ProcTHORInterface:
    """
    Interface for interacting with the ProcThor environments.
    This class provides methods to scrape, download, and extract .tar.gz files containing ProcThor environments.
    Base URL defaults to 'https://user.informatik.uni-bremen.de/~luc_kro/procthor_environments/'
    """

    base_url: str = "https://user.informatik.uni-bremen.de/~luc_kro/procthor_environments/"
    """
    The base URL to scrape for .tar.gz files containing ProcThor environments.
    """

    project_root: str = WorldConfig.project_root
    """
    The root path of the project, used to find resources and other files.
    """

    def get_tarball_links(self) -> List[str]:
        """
        Scrape and return all .tar.gz file links from the specified base URL.

        :return: A list of URLs pointing to .tar.gz files.
        """
        response = requests.get(self.base_url)
        soup = BeautifulSoup(response.text, "html.parser")
        return [
            urljoin(self.base_url, a["href"])
            for a in soup.find_all("a", href=True)
            if a["href"].endswith(".tar.gz")
        ]

    def download_file(self, url: str, filename: str):
        """
        Download a file from a URL to a destination path.

        :param url: The URL of the file to download.
        :param filename: The name of the file to save as.
        """
        response = requests.get(url, stream=True)
        tmp_folder = os.path.join(self.project_root, "tmp")
        os.makedirs(tmp_folder, exist_ok=True)
        with open(os.path.join(tmp_folder, filename), "wb") as f:
            for chunk in response.iter_content(chunk_size=8192):
                if chunk:
                    f.write(chunk)

    def extract_tar(self, filename: str, extract_to: str):
        """
        Extract a .tar.gz archive to the specified directory.

        :param filename: The name of the .tar.gz file to extract.
        :param extract_to: Directory where the contents should be extracted.
        """
        os.makedirs(extract_to, exist_ok=True)
        mode = "r:gz" if filename.endswith(".gz") else "r:"
        with tarfile.open(os.path.join(self.project_root, "tmp", filename), mode) as tar:
            tar.extractall(path=extract_to)

    def sample_environment(self, keep_environment: bool = False):
        """
        Fetch and extract a random selection of environments packed in .tar.gz files from a URL.

        :param keep_environment: If True, the environments will be kept in the resources directory, otherwise they will be
        """
        links = self.get_tarball_links()
        selected_link = random.choice(links)
        if not keep_environment:
            output_dir = os.path.join(self.project_root, "tmp")
        else:
            output_dir = os.path.join(self.project_root, "resources/procthor_environments")

        filename = os.path.basename(selected_link)
        environment_name = os.path.splitext(os.path.splitext(filename)[0])[0]
        self.download_file(selected_link, filename)
        self.extract_tar(filename, output_dir)
        os.remove(os.path.join(self.project_root, "tmp", filename))
        return output_dir, environment_name + "_decomposed"
