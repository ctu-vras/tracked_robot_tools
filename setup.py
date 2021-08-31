#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tracked_robot_tools'],
    package_dir={'': 'src'},
    requires=['rospy']
)

setup(**d)