from setuptools import find_packages
from setuptools import setup

setup(
    name='robot_hardware',
    version='0.0.1',
    packages=find_packages(
        include=('robot_hardware', 'robot_hardware.*')),
)
