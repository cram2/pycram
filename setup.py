from setuptools import find_packages, setup
import os

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
        version='0.0.2',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ] + generate_data_files('share/' + package_name + '/', 'resources'),
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Jonas Dech',
        maintainer_email='jdech@uni-bremen.de',
        description='A Python library for cognitive robot control',
        license='Gplv3',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
            ],
        },
    )