# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_object_recognition_bounding_box'],
    package_dir={'mcr_object_recognition_bounding_box' : 'common/src/mcr_object_recognition_bounding_box'}
)

setup(**d)

