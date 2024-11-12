import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Woodward',
    maintainer_email='mwoodward@gatech.edu',
    description='Skeleton code for the Georgia Tech Autonomous Mobile Robots Lab 4: Controls',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = control.control_node:main',
            'record_path = control.record_path:main',
            'publish_path = control.publish_path:main'
        ],
    },
)
