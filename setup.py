#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['rasberry_perception'],
    package_dir={'': 'src'},
    install_requires=['numpy', 'opencv-python', 'PyYaml', 'rospkg', 'tf', 'Pillow', 'SimpleDataTransport']
)

setup(**setup_args)


