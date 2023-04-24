#!/usr/bin/env python3

from os.path import dirname, abspath, basename
# from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup, find_packages

setup_args = generate_distutils_setup(
    # packages = find_packages()
    packages=[basename(dirname(abspath(__file__)))],
    package_dir={'': 'src'},
    
    ### We dont know if this helps or does anything
    install_requires=['numpy','opencv-python','torch','torchvision'] # https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml
)

setup(**setup_args)