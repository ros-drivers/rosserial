#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosserial_vex_v5'],
    package_dir={'': 'src'},
    )

setup(**d)
