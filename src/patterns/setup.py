from setuptools import setup

package_name = 'patterns'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        'patterns',
        'patterns.behavior_tree',
        'patterns.patterns',
        'patterns.patterns.exploration',
        'patterns.patterns.exploration.nodes',
        'patterns.patterns.systematic_cleaning',
        'patterns.patterns.systematic_cleaning.nodes',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam-warn',
    maintainer_email='adam.o.warn@gmail.com',
    description='Behavior tree manager and pattern implementations for GigaVacuumMobil.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pattern_manager = patterns.behavior_tree.manager_node:main',
        ],
    },
)
