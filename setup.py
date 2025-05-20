import os

from setuptools import setup

current_directory = os.path.dirname(os.path.abspath(__file__))


# Function to read the version from random_events.__version__
def get_version():
    version_file = os.path.join(current_directory, 'src', 'pycram', '__init__.py')
    with open(version_file, 'r') as f:
        for line in f:
            if line.startswith('__version__'):
                delim = '"' if '"' in line else "'"
                return line.split(delim)[1]
    raise RuntimeError("Unable to find version string.")


# Optional project description in README.md:
try:
    with open(os.path.join(current_directory, 'README.md'), encoding='utf-8') as f:
        long_description = f.read()
except Exception:
    long_description = ''

if os.environ.get('ROS_VERSION') == "1":
    ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    # fetch values from package.xml
    setup_args = generate_distutils_setup(
        packages=['pycram'],
        package_dir={'': 'src'}
    )

    setup(**setup_args)
elif os.environ.get('ROS_VERSION') == "2":

    package_name = 'pycram'


    # Iterate through all the files and subdirectories
    # to build the data files array
    def generate_data_files(share_path, dir):
        data_files = []

        for path, _, files in os.walk(dir):
            list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
            data_files.append(list_entry)

        return data_files


    setup(
        name=package_name,
        version=get_version(),
        # packages=find_packages(exclude=['test'], include=['pycram', 'pycrap']),
        package_dir={"": "src"},  # Optional
        packages=["pycram", "pycrap"],
        data_files=[
                       ('share/ament_index/resource_index/packages',
                        ['resource/' + package_name]),
                       ('share/' + package_name, ['package.xml']),
                   ] + generate_data_files('share/' + package_name + '/', 'resources'),
        install_requires=['setuptools', 'pycram_bullet', 'numpy', 'pytransforms3d', 'typing_extensions>=4.10.0',
                          'trimesh==4.6.0', 'random_events>=4.1.0', 'pin==2.7.0', 'transforms3d', 'python-box',
                          'urdf_parser_py', 'networkx', 'pint', 'owlready2>=0.45', 'psutil', 'deprecated'],
        zip_safe=True,
        maintainer='Jonas Dech',
        maintainer_email='jdech@uni-bremen.de',
        description='A Python library for cognitive robot control',
        long_description=long_description,
        long_description_content_type='text/markdown',
        license='Gplv3',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
            ],
        },
    )

else:
    setup(
        name="pycram",
        version=get_version(),
        # packages=find_packages(exclude=['test'], include=['pycram', 'pycrap']),
        package_dir={"": "src"},  # Optional
        packages=["pycram", "pycrap"],
        install_requires=['setuptools', 'pycram_bullet', 'numpy', 'pytransforms3d', 'typing_extensions>=4.10.0',
                          'trimesh==4.6.0', 'random_events>=4.1.0', 'pin==2.7.0', 'transforms3d', 'python-box',
                          'urdf_parser_py', 'networkx', 'pint', 'owlready2>=0.45', 'psutil', 'deprecated'],
        zip_safe=True,
        maintainer='Jonas Dech',
        maintainer_email='jdech@uni-bremen.de',
        description='A Python library for cognitive robot control',
        long_description=long_description,
        long_description_content_type='text/markdown',
        license='Gplv3',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
            ],
        },
    )
