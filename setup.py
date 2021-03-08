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
            include_dirs=["lib/", "lib/sumo/src", "lib/sumo/build/src/",
                          "lib/sumo/build/cmake-build/src/", "/usr/include/fox-1.6/"],
            libraries=["xerces-c", "z", "proj", "FOX-1.6", "X11", "Xext", "freetype", "fontconfig", "Xft", "Xcursor", "Xrender", "Xrandr", "Xfixes", "Xi", "GL", "GLU", "dl", "pthread", "rt", "jpeg", "png", "tiff", "z", "bz2", "lib/sumo/sumo"],
            extra_compile_args=["-DPUGIXML_HEADER_ONLY"]
        ),
        language_level="3"
    )
)