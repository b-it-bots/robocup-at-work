#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_pose_generation', 'mcr_pose_generation_ros'],
    package_dir={'mcr_pose_generation': 'common/src/mcr_pose_generation',
                 'mcr_pose_generation_ros': 'ros/src/mcr_pose_generation_ros'}
)

setup(**d)
