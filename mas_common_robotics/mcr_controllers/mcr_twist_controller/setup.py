#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_twist_controller_ros'],
    package_dir={'mcr_twist_controller_ros': 'ros/src/mcr_twist_controller_ros'}
)

setup(**d)
