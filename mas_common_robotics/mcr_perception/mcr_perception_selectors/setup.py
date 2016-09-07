#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_perception_selectors_ros'],
    package_dir={'mcr_perception_selectors_ros': 'ros/src/mcr_perception_selectors_ros'}
)

setup(**d)
