# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['mcr_moveit_commander_gui'],
    package_dir={'mcr_moveit_commander_gui': 'ros/src/mcr_moveit_commander_gui'},
    requires=['rospy']
)

setup(**setup_args)
