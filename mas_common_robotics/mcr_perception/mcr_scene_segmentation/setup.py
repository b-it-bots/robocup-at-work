# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_scene_segmentation'],
    package_dir={'mcr_scene_segmentation': 'ros/src/mcr_scene_segmentation'}
)

setup(**d)
