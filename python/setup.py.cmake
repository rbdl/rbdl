#!/usr/bin/env python

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from Cython.Build import cythonize

import os
import numpy as np

BASEDIR = os.path.dirname(os.path.abspath(__file__))

extra_params = {}
extra_params['include_dirs'] = [
    '/usr/include',
    BASEDIR,
    np.get_include(),
    '@RBDL_INCLUDE_DIR@',
    '/usr/include/eigen3/'
]

extra_params['extra_compile_args'] = ["-O2", "-Wno-unused-variable"]
extra_params['extra_link_args'] = ["-Wl,-O1", "-Wl,--as-needed"]

extra_params = extra_params.copy()
extra_params['libraries'] = []

extra_params['library_dirs'] = ['/usr/lib', BASEDIR]
extra_params['language'] = 'c++'

if os.name == 'posix':
    extra_params['runtime_library_dirs'] = extra_params['library_dirs']

ext_modules = [
    Extension("rbdl",  ["crbdl.pxd", "rbdl.pyx"], **extra_params),
]

setup(
    name='qpOASES interface',
    cmdclass={'build_ext': build_ext},
    ext_modules=cythonize(ext_modules),
)
