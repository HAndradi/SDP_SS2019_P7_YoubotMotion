#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['motion_coordinators'],
    package_dir={'motion_coordinators': 'src/motion_coordinators'}
)

setup(**d)
