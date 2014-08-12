#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_arm_motions_ros'],
    package_dir={'mcr_arm_motions_ros': 'ros/src'}
)

setup(**d)
