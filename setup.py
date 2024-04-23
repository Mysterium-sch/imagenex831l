#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages
from setuptools import setup

package_name = 'imagenex831l'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    author='Lilian Lamb',
    author_email='lamblily90@gmail.com',
    maintainer='Lilian Lamb',
    maintainer_email='lamblily90@gmail.com',
    keywords=['ROS'],
    description='ROS 2 version of imagenex831l',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sonar = scripts.sonar_node.main',
            'test = scripts.test_sonar.main',
        ],
    },

)
