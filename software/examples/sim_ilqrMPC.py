# Other imports
import numpy as np
import matplotlib.pyplot as plt

# Local imports
from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.ilqr.iLQR_MPC_controller import iLQRMPCController


mass = 0.57288
length = 0.5
damping = 0.15
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 10.0
inertia = mass*length*length

pendulum = PendulumPlant(mass=mass,
                         length=length,
                         damping=damping,
                         gravity=gravity,
                         coulomb_fric=coulomb_fric,
                         inertia=inertia,
                         torque_limit=torque_limit)

sim = Simulator(plant=pendulum)

n_x = 2

dt = 0.02
t_final = 10.0
x0 = np.array([0.0, 0.0])
x0_sim = x0.copy()
goal = np.array([np.pi, 0])

controller = iLQRMPCController(mass=mass,
                               length=length,
                               damping=damping,
                               coulomb_friction=coulomb_fric,
                               gravity=gravity,
                               inertia=inertia,
                               dt=dt,
                               n=50,  # horizon size
                               max_iter=1,
                               break_cost_redu=1e-1,
                               sCu=30.0,
                               sCp=10.0,
                               sCv=1.0,
                               sCen=1.0,
                               fCp=10.0,
                               fCv=1.0,
                               fCen=80.0,
                               dynamics="runge_kutta",
                               n_x=n_x)


controller.set_goal(goal)
# controller.load_initial_guess(filepath="../../../data/trajectories/ilqr/trajectory.csv")
controller.init(x0=x0_sim)

T, X, U = sim.simulate_and_animate(t0=0.0,
                                   x0=x0_sim,
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
ax[2].plot(T, U, label="u")
ax[2].set_xlabel("time [s]")
ax[2].set_ylabel("input torque [Nm]")
ax[2].legend(loc="best")
plt.show()

# Save Trajectory to a csv file to be sent to the motor.

# csv_data = np.vstack((T, np.asarray(X).T[0], np.asarray(X).T[1], U)).T
# np.savetxt("data/ilqr/MPC_trajectory.csv", csv_data, delimiter=',',
#            header="time,pos,vel,torque", comments="")
