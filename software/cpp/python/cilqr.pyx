# distutils: language = c++
# distutils: sources = ../controllers/ilqr/src/ilqr.cpp
# cython: language_level=3

import numpy as np
from cpython cimport Py_INCREF

from cilqr cimport ilqr

cimport numpy as np
np.import_array()

cdef class cppilqr:
    cdef ilqr *il

    def __cinit__(self, N=1000):
        self.il = new ilqr(N)

    def __dealloc__(self):
        del self.il

    def set_parameters(self, nx, nu, integrator_ind, delta_t):
        self.il.set_parameters(nx, nu, integrator_ind, delta_t)

    def set_start(self, pos, vel):
        self.il.set_start(pos, vel)

    def set_goal(self, pos, vel):
        self.il.set_goal(pos, vel)

    def set_cost_parameters(self, double su, double sp, double sv, double sen, double fp, double fv, double fen):
        self.il.set_cost_parameters(su, sp, sv, sen, fp, fv, fen)

    def set_pendulum_parameters(self, m, l, d, cf, g, I, tl):
        self.il.set_pendulum_parameters(m, l, d, cf, g, I, tl)

    def set_u_init_traj(self, u):
        cdef np.ndarray[double, ndim=1, mode="c"] uu
        uu = u
        self.il.set_u_init_traj(&uu[0])

    def set_x_init_traj(self, p, v):
        cdef np.ndarray[double, ndim=1, mode="c"] pp
        cdef np.ndarray[double, ndim=1, mode="c"] vv
        pp = p
        vv = v
        self.il.set_x_init_traj(&pp[0], &vv[0])

    def run_ilqr(self, max_iter, break_cost_redu, regu_init):
        self.il.run_ilqr(max_iter, break_cost_redu, regu_init)

    def get_u_traj(self):
        cdef np.npy_intp shape[1]
        cdef np.ndarray ar
        shape[0] = self.il.get_N() - 1
        ar = np.PyArray_SimpleNewFromData(1, shape, np.NPY_DOUBLE, <void*>self.il.get_u_traj())
        Py_INCREF(self)
        np.PyArray_SetBaseObject(ar, self)
        return ar

    def get_p_traj(self):
        cdef np.npy_intp shape[1]
        cdef np.ndarray ar
        shape[0] = self.il.get_N()
        ar = np.PyArray_SimpleNewFromData(1, shape, np.NPY_DOUBLE, <void*>self.il.get_p_traj())
        Py_INCREF(self)
        np.PyArray_SetBaseObject(ar, self)
        return ar

    def get_v_traj(self):
        cdef np.npy_intp shape[1]
        cdef np.ndarray ar
        shape[0] = self.il.get_N()
        ar = np.PyArray_SimpleNewFromData(1, shape, np.NPY_DOUBLE, <void*>self.il.get_v_traj())
        Py_INCREF(self)
        np.PyArray_SetBaseObject(ar, self)
        return ar
