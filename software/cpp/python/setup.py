from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
import numpy as np

ilqr_extension = Extension(
    name="cpppendulumilqr",
    sources=["cilqr.pyx"],
    libraries=["ilqr"],
    library_dirs=["../controllers/ilqr/obj", "../"],
    include_dirs=[
        "../controllers/ilqr/obj",
        "../",
        np.get_include(),
    ],
)
setup(name="cppilqr", ext_modules=cythonize([ilqr_extension]))
