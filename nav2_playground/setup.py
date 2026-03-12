from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'nav2_playground'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nav2_playground',
    maintainer_email='dev@example.com',
    description=(
        'A lightweight navigation playground for testing controllers,'
        ' planners, and behaviors.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'playground_test_runner = nav2_playground.test_runner:main',
        ],
    },
)
