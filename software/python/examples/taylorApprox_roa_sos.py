import matplotlib.pyplot as plt
import numpy as np
from pydrake.symbolic import TaylorExpand, Evaluate
from pydrake.all import Variable

from simple_pendulum.controllers.lqr.lqr_controller import LQRController
from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.controllers.lqr.roa.utils import PendulumPlantApprox
from simple_pendulum.simulation.simulation import Simulator

mass = 0.57288
length = 0.5
damping = 0.15
gravity = 9.81
coulomb_fric = 0.0
inertia = mass*length*length
torque_limit = 8.0
x_i = [0.5,0.1]

controller = LQRController(mass=mass,
                           length=length,
                           damping=damping,
                           gravity=gravity,
                           torque_limit=torque_limit)

pendulum = PendulumPlant(mass=mass,
                         length=length,
                         damping=damping,
                         gravity=gravity,
                         torque_limit=torque_limit)

controller.set_goal([np.pi, 0])
controller.set_clip()
dt = 0.01
t_final = 3.0

sim = Simulator(plant=pendulum)
T, X, U = sim.simulate(t0=0.0,
                        x0=x_i,
                        tf=t_final,
                        dt=dt,
                        controller=controller,
                        integrator="runge_kutta"
                        )

theta = np.asarray(X).T[0]
theta_dot = np.asarray(X).T[1]
theta_Ddot = (U- mass* gravity * length * np.sin(theta) - damping * theta_dot) / inertia

fig, ax = plt.subplots(3, 1, figsize=(18, 8), sharex="all")
ax[0].plot(T, theta, label="non-linear", linewidth = 5, color = "black", alpha = 0.2)
ax[0].set_ylabel("angle [rad]")
ax[1].plot(T, theta_dot, label="non-linear", linewidth = 5,color = "black", alpha = 0.2)
ax[1].set_ylabel("angular velocity [rad/s]")
ax[2].plot(T, theta_Ddot, label="non-linear", linewidth = 5,color = "black", alpha = 0.2)
ax[2].set_ylabel("angular accelleration [rad/s^2]")
ax[2].set_xlabel("time [s]")

max_order = 4
err = []
for j in range(1,max_order+1):                                
    pendulum_approx = PendulumPlantApprox(mass=mass,
                                      length=length,
                                      damping=damping,
                                      gravity=gravity,
                                      torque_limit=torque_limit,
                                      taylorApprox_order = j)

    sim_approx = Simulator(plant=pendulum_approx)
    T_approx, X_approx, U_approx = sim_approx.simulate(t0=0.0,
                                                    x0=x_i,
                                                    tf=t_final,
                                                    dt=dt,
                                                    controller=controller,
                                                    integrator="runge_kutta"
                                                    )

    
    
    theta_approx = np.asarray(X_approx).T[0]
    theta_dot_approx = np.asarray(X_approx).T[1]

    x0 = Variable("theta")
    Tsin_exp = TaylorExpand(np.sin(x0), {x0: np.pi},j)
    Tsin = np.empty(len(theta_approx))
    for i in range(len(theta_approx)):
        Tsin[i] = Tsin_exp.Evaluate({x0 : theta_approx[i]})

    theta_Ddot_approx = (U_approx - mass* gravity * length * Tsin - damping * theta_dot_approx) / inertia

    ax[0].plot(T, theta_approx, label=f"order {j}", dashes = [6,2])
    ax[1].plot(T, theta_dot_approx, label=f"order {j}",dashes = [6,2])
    ax[2].plot(T, theta_Ddot_approx, label=f"order {j}",dashes = [6,2])

plt.suptitle("Taylor approximations of the closed-loop dynamics")
plt.legend(loc = "right")
plt.show()