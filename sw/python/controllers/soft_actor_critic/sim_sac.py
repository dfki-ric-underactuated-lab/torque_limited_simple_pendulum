# Other Imports
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# Set path for local imports
import site
site.addsitedir('../..')

# Local imports
from model.pendulum_plant import PendulumPlant
from simulation.simulation import Simulator
from controllers.soft_actor_critic.sac_controller import SacController


mass = 0.57288
length = 0.5
damping = 0.15
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 10.0
inertia = mass*length*length

# get the controller
controller = SacController()

# get the simulator
torque_limit = controller.params['torque_limit']
mass = 0.546  # 0.546
length = 0.45  # 0.45
damping = 0.16
gravity = 9.81
coulomb_fric = 0
inertia = mass*length**2

pendulum = PendulumPlant(mass=mass,
                         length=length,
                         damping=damping,
                         gravity=gravity,
                         coulomb_fric=coulomb_fric,
                         inertia=inertia,
                         torque_limit=torque_limit)

sim = Simulator(plant=pendulum)

# simulate
x0_sim = [0.0, 0.0]  #[np.random.rand()*0.01, np.random.rand()*0.01]
dt = torque_limit = float(controller.params['dt'])
t_final = 10
integrator = controller.params['integrator']

T, X, U = sim.simulate(t0=0.0,
                       x0=x0_sim,
                       tf=t_final,
                       dt=dt,
                       controller=controller,
                       integrator=integrator)

fig, ax = plt.subplots(3, 1, figsize=(18, 6), sharex="all")

ax[0].plot(T, np.asarray(X).T[0], label="theta")
ax[0].set_ylabel("angle [rad]")
ax[0].legend(loc="best")
ax[1].plot(T, np.asarray(X).T[1], label="theta dot")
ax[1].set_ylabel("angular velocity [rad/s]")
ax[1].legend(loc="best")
ax[2].plot(T, U, label="u")
ax[2].set_xlabel("time [s]")
ax[2].set_ylabel("input torque [Nm]")
ax[2].legend(loc="best")
plt.show()

a = None
