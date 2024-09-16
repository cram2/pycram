# import wget
# import requests
# import bs4 as BeautifulSoup
# from matplotlib.style import available
# from wget import download


import os
import requests
from urllib.parse import urljoin
from bs4 import BeautifulSoup

# Function to download a file from a URL
def download_file(base_url, url, folder):
    tree_structure=url.replace(base_url, '')
    full_storage_path= os.path.join(folder, tree_structure)
    if not os.path.exists(full_storage_path[:full_storage_path.rfind('/')+1]):
        os.makedirs(full_storage_path[:full_storage_path.rfind('/')+1], exist_ok=True)
    local_filename = os.path.join(folder, url.split('/')[-1])
    # Send HTTP GET request to fetch the file
    with requests.get(url, stream=True) as r:
        r.raise_for_status()
        # Write the content to a file
        with open(full_storage_path, 'wb') as f:
            for chunk in r.iter_content(chunk_size=8192):
                f.write(chunk)
    return local_filename

# Function to get the list of files in a directory on the web server
def get_files_list(base_url):
    print("get_files_list({})".format(base_url))
    # Send GET request to the URL
    response = requests.get(base_url)
    # Parse the HTML response
    soup = BeautifulSoup(response.text, 'html.parser')
    # Find all links on the page
    links = soup.find_all('a')
    file_list = set()
    files_urls= set()
    for link in links:
        if link.get('href').endswith('/') and not link.get('href').startswith('/'):
            files_urls=files_urls.union(get_files_list(base_url + link['href']))

        elif link.get('href').endswith(('.usda', '.urdf', '.stl', '.usd', '.hdr')):
            files_urls.add(urljoin(base_url, link['href']))
        else:
            print("Ignored File: {}".format(link.get('href')))
            # Filter links that point to files
            # file_urls = [urljoin(base_url, link['href']) for link in links if link['href'].endswith()]
    return files_urls

# Main function to download all files from a directory
def download_all_files(base_url, folder):
    # Ensure the folder exists
    os.makedirs(folder, exist_ok=True)
    # Get the list of files
    files = get_files_list(base_url)
    # create_folder_structure(base_url,folder,files)
    # Download each file
    for file_url in files:
        print(f"Downloading {file_url}...")
        download_file(base_url,file_url, folder)
    print("All files downloaded.")

# Expect urdf files to be in hte folder below base_url otherwise I stop searching
def get_all_environments(base_url):
    urdf_files= []
    for folder in os.listdir(base_url):
        for file in os.listdir(os.path.join(base_url, folder)):
            if file.endswith('.urdf'):
                urdf_files.append(os.path.join(base_url, folder, file))
    return urdf_files

#
# # Example usage
# base_url = 'http://192.168.102.44/environment/'
# download_folder = "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/"
# download_all_files(base_url, download_folder)
