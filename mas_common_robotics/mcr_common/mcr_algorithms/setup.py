#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mcr_algorithms'],
   package_dir={'mcr_algorithms': 'common/src'}
)

setup(**d)
