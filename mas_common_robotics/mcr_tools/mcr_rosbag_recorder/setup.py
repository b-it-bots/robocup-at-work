#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_rosbag_recorder'],
    package_dir={'mcr_rosbag_recorder': 'ros/src/mcr_rosbag_recorder'}
)

setup(**d)
