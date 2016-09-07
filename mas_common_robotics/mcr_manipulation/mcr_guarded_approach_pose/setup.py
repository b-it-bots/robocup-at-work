#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_guarded_approach_pose_ros'],
    package_dir={'mcr_guarded_approach_pose_ros': 'ros/src/mcr_guarded_approach_pose_ros'}
)

setup(**d)
