from setuptools import find_packages, setup

package_name = 'reactive_autonomy'

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
    maintainer='Matthew Woodward',
    maintainer_email='mwoodward@gatech.edu',
    description='Skeleton code for the Georgia Tech Autonomous Mobile Robots Lab 2: Reactive Autonomy',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'automatic_braking = reactive_autonomy.automatic_braking:main',
            'gap_following = reactive_autonomy.gap_following:main'
        ],
    },
)
