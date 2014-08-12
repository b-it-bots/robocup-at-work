#!/usr/bin/env python

import distutils.core
import catkin_pkg.python_setup

d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=['mcr_object_detection'],
   package_dir={'mcr_object_detection': 'ros/src'}
)

distutils.core.setup(**d)
