#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['grasberry_manipulation', 'grasberry_manipulation_pkg', 'arm_3dof'],
    package_dir={'': 'src'},
    requires=['numpy', 'opencv-python', 'PyYaml', 'rospkg', 'tf']
)

setup(**setup_args)


