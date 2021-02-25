from setuptools import setup, Extension
from Cython.Build import cythonize

setup(ext_modules = cythonize(Extension(
    "libiintersection",
    sources=["libiintersection.pyx"],
    language="c++",
    extra_compile_args=["-DPUGIXML_HEADER_ONLY", "-Ilib/", "-Ilib/sumo/src/", "-Ilib/sumo/build/src/", "-Ilib/sumo/build/cmake-build/src/"]
)))