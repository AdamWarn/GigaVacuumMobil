from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cleaning_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam-warn',
    maintainer_email='adam.o.warn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_generator = cleaning_planner.waypoint_generator:main',
            'coverage_planner_node = cleaning_planner.coverage_planner_node:main',
            'coverage_executor_node = cleaning_planner.coverage_executor_node:main',
        ],
    },
)
