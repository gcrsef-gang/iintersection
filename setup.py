from setuptools import setup
from Cython.Build import cythonize

setup(ext_modules = cythonize(
    "libiintersection.pyx",
    language_level="3",
    language="c++"
))