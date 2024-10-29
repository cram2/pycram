# import wget
# import requests
# import bs4 as BeautifulSoup
# from matplotlib.style import available
# from wget import download

import fnmatch
import os
import requests
from urllib.parse import urljoin
from bs4 import BeautifulSoup
import rospkg

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram')

class ProcThorInterface:
    def __init__(self,base_url="http://procthor.informatik.uni-bremen.de:5000",source_folder=os.path.join(rospack.get_path('pycram'),"resources/tmp/")):
        self.base_url=base_url
        self.source_folder=source_folder
        self.stored_environments=[]
        self.stored_environments.extend(self.get_all_environments_stored_below_folder(self.source_folder))

    def _download_file(self, base_url, full_url, folder):
        print("base_url:{} full_url:{} folder:{}".format(base_url,full_url,folder))
        tree_structure = full_url.replace(base_url, '')
        # print("tree_structure:{}".format(tree_structure))
        if tree_structure.startswith("/"):
            tree_structure = tree_structure[1:]
        # print("folder:{} tree_structure:{} joined_path:{}".format(folder,tree_structure,os.path.join(folder, tree_structure)))
        full_storage_path = os.path.join(folder, tree_structure)
        # print("tree_structure:{} full_storage_path:{}".format(tree_structure,full_storage_path))
        if not os.path.exists(full_storage_path[:full_storage_path.rfind('/') + 1]):
            print(full_storage_path[:full_storage_path.rfind('/') + 1])
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
    def _get_files_list(self, base_url):
        print("get_files_list({})".format(base_url))
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
                # print(10*'§')
                # print(os.path.join(base_url, link['href']))
                # print(10 * '§')
                files_urls = files_urls.union(self._get_files_list(os.path.join(base_url, link['href'])))

            elif link.get('href').endswith(('.usda', '.urdf', '.stl', '.usd', '.hdr')):
                # print(10 * '!')
                # print(os.path.join(base_url, link['href']))
                # print(urljoin(base_url, link['href']))
                # print(10 * '!')
                files_urls.add(os.path.join(base_url, link['href']))
            else:
                print("Ignored File: {}".format(link.get('href')))
                # Filter links that point to files
                # file_urls = [urljoin(base_url, link['href']) for link in links if link['href'].endswith()]
        return files_urls

    # Main function to download all files from a directory
    def download_all_files_from_URL(self, base_url, folder):
        # Ensure the folder exists
        os.makedirs(folder, exist_ok=True)
        # Get the list of files
        files = self._get_files_list(base_url)
        # create_folder_structure(base_url,folder,files)
        # Download each file
        print(files)
        for file_url in files:
            print(f"Downloading {file_url}...")
            self._download_file(base_url, file_url, folder)
        print("All files downloaded.")

    # Returns a list of all the urdf files in a given folder structure if non is given uses the base_source_folder
    # only look for urdf, but can be easly adapted
    def get_all_environments_stored_below_folder(self, source_folder):
        urdf_files = []
        urdf_file={}
        #ToDo:apritrarry depth
        for root, dirs, files in os.walk(source_folder):
            for filename in fnmatch.filter(files, '*.urdf'):
                urdf_file["name"]=filename.rsplit('.', 1)[0]
                urdf_file["storage_place"] = os.path.join(root, filename)
                urdf_files.append(urdf_file.copy())
        return urdf_files

    def download_one_random_environment(self):
        endpoint="GetRandomEnvironment"
        full_url=os.path.join(self.base_url,endpoint)
        response = requests.get(full_url)
        self.download_all_files_from_URL(response.json()["storage_place"],self.source_folder)
        self.stored_environments.append(response.json())
        return response.json()

    def download_num_random_environment(self, num_of_environments):
        endpoint="GetNumEnvironment"
        full_url=os.path.join(self.base_url,endpoint,str(num_of_environments))
        response = requests.get(full_url)
        for env in response.json():
            print(env)
            self.download_all_files_from_URL(env["storage_place"],self.source_folder)
            self.stored_environments.append(env)
        return response.json()



procThorInterface=ProcThorInterface(base_url="http://127.0.0.1:5000")
# print(procThorInterface.stored_environments)
# print(10*'#')
# print(procThorInterface.download_num_random_environment(3))
# print(10*'#')
# print(procThorInterface.stored_environments)
# # print(procThorInterface.download_num_random_environment(3))
# for key in procThorInterface.stored_environments:
#     print(key)
#     print(key,procThorInterface.stored_environments[key])