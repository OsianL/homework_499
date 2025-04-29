from setuptools import find_packages, setup

package_name = 'hw3'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    
	data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    
	maintainer='osianl',
    maintainer_email='leahyo@oregonstate.edu',
    description='Homework 3 for ROB499',
    license='MIT',

    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			# Node for Sending TestPackets
			'data_sender = hw3.data_sender:main',
			#Node for recieving Testpackets and rebroadcasting latency
			'data_receiver = hw3.data_receiver:main',
			#Clients for enabling and disabling data sending:
			'send_enable = hw3.send_service_client:send_data_enable',
			'send_disable = hw3.send_service_client:send_data_disable',
			#Clients for enabling and disabling data logging:
			'log_enable = hw3.logenable_service_client:enable_logging',
			'log_disable = hw3.logenable_service_client:disable_logging'
        ],
    },
)
