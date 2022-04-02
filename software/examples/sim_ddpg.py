import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.ddpg.ddpg_controller import ddpg_controller


mass = 0.57288
length = 0.5
damping = 0.15
gravity = 9.81
coulomb_fric = 0.02
torque_limit = 1.5
inertia = mass*length*length

dt = 0.01
t_final = 10.0

pendulum = PendulumPlant(mass=mass,
                         length=length,
                         damping=damping,
                         gravity=gravity,
                         coulomb_fric=coulomb_fric,
                         inertia=inertia,
                         torque_limit=torque_limit)


model_path = "../../../data/models/ddpg_model/actor"
controller = ddpg_controller(model_path=model_path,
                             torque_limit=torque_limit,
                             state_representation=3)


sim = Simulator(plant=pendulum)

T, X, U = sim.simulate_and_animate(t0=0.0,
                                   x0=[-0.1, 0.0],
                                   tf=t_final,
                                   dt=dt,
                                   controller=controller,
                                   integrator="runge_kutta")

fig, ax = plt.subplots(3, 1, figsize=(18, 6), sharex="all")

ax[0].plot(T, np.asarray(X).T[0], label="theta")
ax[0].set_ylabel("angle [rad]")
ax[0].legend(loc="best")
ax[1].plot(T, np.asarray(X).T[1], label="theta dot")
ax[1].set_ylabel("angular velocity [rad/s]")
ax[1].legend(loc="best")
ax[2].plot(T, np.asarray(U).flatten(), label="u")
ax[2].set_xlabel("time [s]")
ax[2].set_ylabel("input torque [Nm]")
ax[2].legend(loc="best")
plt.show()
