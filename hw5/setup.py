from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'hw5'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name), glob('*.rviz')),
		(os.path.join('share',package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osianl',
    maintainer_email='leahyo@oregonstate.edu',
    description='Submission for ROB499 HW5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'laser_filter = hw5.laser_filter:main',
			'people_detector = hw5.people_detector:main',
			'personal_space = hw5.personal_space:main',
			'camera_capture = hw5.camera_capture:main'
		],
    },
)
