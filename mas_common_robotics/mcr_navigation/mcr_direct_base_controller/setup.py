#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_direct_base_controller_ros'],
    package_dir={'mcr_direct_base_controller_ros': 'ros/src/mcr_direct_base_controller_ros'}
)

setup(**d)
