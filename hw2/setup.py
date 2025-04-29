from setuptools import find_packages, setup

package_name = 'hw2'

setup(
    name=package_name,
    version='0.0.0',
    
	packages=find_packages(exclude=['test']),
    
	data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    
	maintainer='osianl',
    maintainer_email='osianl@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    
	tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #A node which publishes a 1hz sinusoid based on the current epoch time
			'oscope = hw2.oscope:normal_wave',
			'slow_wave = hw2.oscope:slow_wave',
			'fast_wave = hw2.oscope:fast_wave',
			#A node which limits the incoming signal to +/- 0.5:
			'limiter = hw2.limiter:limit'
        ],
    },
)
