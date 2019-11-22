#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['rasberry_perception', 'rasberry_perception_pkg', 'deep_learning_ros'],
    package_dir={'': 'src'},
    requires=['numpy', 'opencv-python', 'PyYaml', 'rospkg', 'tf', 'Pillow', 'SimpleDataTransport']
)

setup(**setup_args)


