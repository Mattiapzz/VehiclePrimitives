#!/usr/bin/env python3

from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext
import glob

# Collect all C++ source files from the main library.
# The paths are relative to this setup.py file.
vprim_sources = glob.glob("../src/*.cc")

ext_modules = [
    Pybind11Extension(
        "vprim_py",  # The name of the Python module
        # The source files for the extension: the bindings plus all library sources.
        ["bindings.cc"] + vprim_sources,
        # Include directories. pybind11.setup_helpers automatically adds pybind11's includes.
        include_dirs=[
            "../src",  # Needed to find "include/utility.hh", etc.
        ],
        language='c++',
        cxx_std=17,
    ),
]

setup(
    name="vprim_py",
    version="0.1.0",
    author="Mattia Piazza",
    author_email="mattia.piazza@unitn.it",
    description="Python bindings for the Vehicle Primitives library",
    long_description="Python bindings for the C++ Vehicle Primitives library, providing tools for vehicle trajectory generation and optimization.",
    ext_modules=ext_modules,
    # Use the custom build extension command from pybind11
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
)