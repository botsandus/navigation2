from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'nav2_nav_tester'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scenarios'), glob('scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nav2_nav_tester',
    maintainer_email='dev@example.com',
    description='YAML-driven navigation test framework with pluggable backends.',
    license='Apache-2.0',
    tests_require=['pytest', 'launch_testing'],
    entry_points={
        'console_scripts': [
            'nav_test_runner = nav2_nav_tester.runner:main',
        ],
    },
)
