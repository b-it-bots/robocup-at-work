#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_trajectory_generation_ros'],
    package_dir={'mcr_trajectory_generation_ros': 'ros/src/mcr_trajectory_generation_ros'}
)

setup(**d)
