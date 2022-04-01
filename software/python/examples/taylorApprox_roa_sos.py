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

# non-linear dynamics simulation
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

# choice of the max order for the Taylor expansion
max_order = 7
orders = range(1,max_order+1,2)

fig, ax = plt.subplots(3, 1, figsize=(18, 8), sharex="all")
ax[0].plot(T, theta, label="non-linear", linewidth = 5, color = "black", alpha = 0.2)
ax[0].set_ylabel("angle [rad]")
ax[1].plot(T, theta_dot, label="non-linear", linewidth = 5,color = "black", alpha = 0.2)
ax[1].set_ylabel("angular velocity [rad/s]")
ax[2].plot(T, theta_Ddot, label="non-linear", linewidth = 5,color = "black", alpha = 0.2)
ax[2].set_ylabel(r"angular accelleration [rad/$s^2$]")
ax[2].set_xlabel("time [s]")
plt.suptitle("Simulated Taylor approximations of the closed-loop dynamics")

# dynamic figure structure
o_c_max = 0
if (len(orders)%2 ==0):
    o_c_max = round(len(orders)/2)
    fig1, ax1 = plt.subplots(2, o_c_max, figsize=(len(orders)*3, len(orders)*3), sharex="all")
else: 
    o_c_max = round(len(orders)/2 + len(orders)%2)
    fig1, ax1 = plt.subplots(2, o_c_max, figsize=(len(orders)*3, len(orders)*3), sharex="all")

plt.suptitle("Taylor approximations of the closed-loop dynamics: State dependent error")

o_r = 0
o_c = 0
for k in orders:  
    # k-order Taylor approximated dynamics simulation                           
    pendulum_approx = PendulumPlantApprox(mass=mass,
                                      length=length,
                                      damping=damping,
                                      gravity=gravity,
                                      torque_limit=torque_limit,
                                      taylorApprox_order = k)

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
    Tsin_exp = TaylorExpand(np.sin(x0), {x0: np.pi}, k)
    Tsin = np.empty(len(theta_approx))
    for i in range(len(theta_approx)):
        Tsin[i] = Tsin_exp.Evaluate({x0 : theta_approx[i]})

    theta_Ddot_approx = (U_approx - mass* gravity * length * Tsin - damping * theta_dot_approx) / inertia

    ax[0].plot(T, theta_approx, label=f"order {k}", dashes = [6,2])
    ax[1].plot(T, theta_dot_approx, label=f"order {k}",dashes = [6,2])
    ax[2].plot(T, theta_Ddot_approx, label=f"order {k}",dashes = [6,2])
 
    # state dependent error in the approximated dynamics
    theta = np.linspace(0,2*np.pi,100)
    theta_dot = np.linspace(-10,10,100)
    Tsin = np.empty(len(theta_approx))
    for i in range(len(theta_approx)):
        Tsin[i] = Tsin_exp.Evaluate({x0 : theta_approx[i]})

    err = np.zeros((len(theta_dot),len(theta)))
    for i in range(len(theta)):
        for j in range(len(theta_dot)):
            U = controller.K.dot([theta[i]-np.pi,theta_dot[j]])
            theta_Ddot = (U - mass* gravity * length * np.sin(theta[i]) - damping * theta_dot[j]) / inertia
            theta_Ddot_approx = (U - mass* gravity * length * Tsin[i] - damping * theta_dot_approx[j]) / inertia
            err[j][i] = np.abs(theta_Ddot_approx-theta_Ddot)**2

    ax1[o_r,o_c].set_title(f"Order {k}")
    ax1[o_r,o_c].set_xlabel("angle [rad]")
    ax1[o_r, o_c].set_ylabel("angular velocity [rad/s]")
    c = ax1[o_r, o_c].pcolormesh(theta, theta_dot,err, cmap = 'coolwarm', vmin = 0, vmax = 4000 )
    fig1.colorbar(c, ax = ax1[o_r,o_c])
    
    if (o_c == o_c_max-1):
        o_r = o_r+1
        o_c = - 1
    o_c = o_c+1

ax[1].legend(loc = "center right")
plt.show()