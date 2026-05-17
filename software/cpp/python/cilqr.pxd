# distutils: language = c++
# cython: language_level=3

#cimport cython
from libcpp.vector cimport vector

cdef extern from "../model/src/pendulum_plant.cpp":
    pass

#cdef extern from "../pendulum_plant/src/pendulum_plant.hpp":
#    pass

cdef extern from "../simulator/src/simulator.cpp":
    pass

#cdef extern from "../simulator/src/simulator.hpp":
#    pass

#cdef extern from "../ilqr/src/ilqr.cpp":
#    pass

cdef extern from "../controllers/ilqr/src/ilqr.hpp":
    cdef cppclass ilqr:
        ilqr() except +
        ilqr(int N) except +
        #ilqr()
        #ilqr(int)
        void set_parameters(int nx, int nu, int integrator_ind, double delta_t)
        void set_start(double pos, double vel)
        void set_goal(double pos, double vel)
        void set_cost_parameters(double su, double sp, double sv, double sen, double fp, double fv, double fen)
        void set_pendulum_parameters(double m, double l, double d, double cf, double g, double I, double tl)
        void set_u_init_traj(double u[])
        void set_x_init_traj(double p[], double v[])
        void run_ilqr(int max_iter, double break_cost_redu, double regu_init)
        #vector[double] * get_u_traj()
        double *get_u_traj()
        double *get_p_traj()
        double *get_v_traj()
        int get_N()
        int N
