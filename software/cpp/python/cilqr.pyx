# distutils: language = c++
# distutils: sources = ../controllers/ilqr/src/ilqr.cpp
# cython: language_level=3

import numpy as np
#from eigency.core cimport *
from cpython cimport PyObject, Py_INCREF
from libcpp.vector cimport vector
from libc.stdlib cimport free

from cilqr cimport ilqr

cimport numpy as np
np.import_array()

cdef class ArrayWrapper:
    cdef void* data_ptr
    cdef int size

    cdef set_data(self, int size, void* data_ptr):
        """ Set the data of the array
        This cannot be done in the constructor as it must recieve C-level
        arguments.
        Parameters:
        -----------
        size: int
            Length of the array.
        data_ptr: void*
            Pointer to the data            
        """
        self.data_ptr = data_ptr
        self.size = size

    def __array__(self):
        """ Here we use the __array__ method, that is called when numpy
            tries to get an array from the object."""
        #cdef np.npy_intp shape[1]
        #shape[0] = <np.npy_intp> self.size
        cdef np.npy_intp shape[1]
        shape[0] = <np.npy_intp> self.size
        # Create a 1D array, of length 'size'
        ndarray = np.PyArray_SimpleNewFromData(1, shape,
                                               np.NPY_DOUBLE, self.data_ptr)
        return ndarray

    def __dealloc__(self):
        """ Frees the array. This is called by Python when all the
        references to the object are gone. """
        free(<void*>self.data_ptr)

cdef class cppilqr:
    #cdef ilqr il
    cdef ilqr *il
    #cdef ilqr il = new ilqr()
    #cdef u_traj_d = new vector[double]()

    def __cinit__(self, N=1000):
        # cdef int n = N
        self.il = new ilqr(N)
        #self.il.N = N

    #def __dealloc__(self):
    #    del self.il

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
        N = self.il.get_N()
        cdef double *vec
        cdef np.ndarray ar
        vec = self.il.get_u_traj()

        array_wrapper = ArrayWrapper()
        array_wrapper.set_data(N-1, <void*> vec)
        ar = np.array(array_wrapper, copy=False)
        ar.base = <PyObject*> array_wrapper
        Py_INCREF(array_wrapper)
        return ar

    def get_p_traj(self):
        N = self.il.get_N()
        cdef double *vec
        cdef np.ndarray ar
        vec = self.il.get_p_traj()

        array_wrapper = ArrayWrapper()
        array_wrapper.set_data(N, <void*> vec)
        ar = np.array(array_wrapper, copy=False)
        ar.base = <PyObject*> array_wrapper
        Py_INCREF(array_wrapper)
        return ar

    def get_v_traj(self):
        N = self.il.get_N()
        cdef double *vec
        cdef np.ndarray ar
        vec = self.il.get_v_traj()

        array_wrapper = ArrayWrapper()
        array_wrapper.set_data(N, <void*> vec)
        ar = np.array(array_wrapper, copy=False)
        ar.base = <PyObject*> array_wrapper
        Py_INCREF(array_wrapper)
        return ar
