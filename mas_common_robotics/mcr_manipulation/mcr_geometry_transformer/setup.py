#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['geometry_transformer_util', 'mcr_geometry_transformer'],
   package_dir={'geometry_transformer_util': 'common/src/geometry_transformer_util',
                'mcr_geometry_transformer': 'ros/src/mcr_geometry_transformer'}
)

setup(**d)
