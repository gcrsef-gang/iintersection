from setuptools import find_packages, setup, Extension
from Cython.Build import cythonize

setup(
    name="iintersection",
    packages=find_packages(),
    ext_modules=cythonize(
        Extension(
            "libiintersection",
            sources=["libiintersection.pyx"],
            language="c++",
            extra_compile_args=["-DPUGIXML_HEADER_ONLY", "-Ilib/", "-Ilib/sumo/src/", "-Ilib/sumo/build/src/", "-Ilib/sumo/build/cmake-build/src/", "-I/usr/include/fox-1.6/"]
        ),
        language_level="3"
    )
)