from setuptools import find_packages, setup

#import for running launch files
import os
from glob import glob

package_name = 'hw4'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osianl',
    maintainer_email='leahyo@oregonstate.edu',
    description='Homework 4 of ROB 499 at Oregon State University',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'oscope = hw4.oscope:run_node',
			'nasa = hw4.nasa:main',
			'nasa_client_launch = hw4.nasa_client:without_cancel',
			'nasa_client_launch_cancel = hw4.nasa_client:with_cancel'
        ],
    },
)
