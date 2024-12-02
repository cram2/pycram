import os
if os.environ.get('ROS_VERSION', None) == '1':

	## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

	from distutils.core import setup
	from catkin_pkg.python_setup import generate_distutils_setup

	# fetch values from package.xml
	setup_args = generate_distutils_setup(
		packages=['pycram'],
		package_dir={'': 'src'}
	)

	setup(**setup_args)
elif os.environ.get('ROS_VERSION', None) == '2':
	from setuptools import find_packages, setup

	package_name = 'pycram'

	setup(
		name=package_name,
		version='0.0.2',
		packages=find_packages(exclude=['test']),
		data_files=[
			('share/ament_index/resource_index/packages',
			 ['resource/' + package_name]),
			('share/' + package_name, ['package.xml']),
		],
		install_requires=['setuptools'],
		zip_safe=True,
		maintainer='Jons Dech',
		maintainer_email='jdech@uni-bremen.de',
		description='TODO: Package description',
		license='GPLv3',
		tests_require=['pytest'],
		entry_points={
			'console_scripts': [
			],
		},
	)