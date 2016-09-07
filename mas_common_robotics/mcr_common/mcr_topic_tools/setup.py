#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_topic_tools_ros'],
    package_dir={'mcr_topic_tools_ros': 'ros/src/mcr_topic_tools_ros'}
)

setup(**d)
