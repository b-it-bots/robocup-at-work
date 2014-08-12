## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_object_recognition_mean_circle'],
    package_dir={'mcr_object_recognition_mean_circle' : 'common/src'}
)

setup(**d)

