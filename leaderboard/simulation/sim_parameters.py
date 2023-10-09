import numpy as np

mass = 0.57288
length = 0.5
damping = 0.10
gravity = 9.81
coulomb_fric = 0.093
torque_limit = 2.0
inertia = mass * length * length

dt = 0.01
t_final = 10.0

t0 = 0.0
x0 = [0., 0.]
goal = np.array([np.pi, 0])

integrator = "runge_kutta"
