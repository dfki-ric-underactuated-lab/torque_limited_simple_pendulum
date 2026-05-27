import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.discrete_graph.graph_controller import GraphController
from simple_pendulum.utilities.process_data import save_trajectoryTXU

mass = 0.57288
length = 0.5
damping = 0.1
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 1.5
inertia = mass * length * length

dt = 0.01
t_final = 10.0

graph_dt = 0.1
graph_pos_res = np.deg2rad(1.0)
graph_vel_res = graph_pos_res / graph_dt
graph_vel_max = 50 * graph_vel_res
print(graph_dt, graph_pos_res, graph_vel_res, graph_vel_max)


pendulum = PendulumPlant(
    mass=mass,
    length=length,
    damping=damping,
    gravity=gravity,
    coulomb_fric=coulomb_fric,
    inertia=inertia,
    torque_limit=torque_limit,
)


# policy_path = "../../../data/models/graph/L0.4/dijkstra_35868.csv"
policy_path = "../../../data/models/graph/L0.5/dijkstra_36360.csv"
controller = GraphController(
    graph_csv_path=policy_path,
    pos_res=graph_pos_res,
    vel_res=graph_vel_res,
    vel_max=graph_vel_max,
)


sim = Simulator(plant=pendulum)

T, X, U = sim.simulate_and_animate(
    # T, X, U = sim.simulate(
    t0=0.0,
    x0=[0.0, 0.0],
    tf=t_final,
    dt=dt,
    controller=controller,
    integrator="runge_kutta",
)
save_trajectoryTXU("log_data/sim_graph_controller/trajectory.csv", T, X, U)

# np.savetxt(
#     "discrete_graph_simulation_swingup.csv",
#     np.column_stack([T, X, U]),
#     delimiter=",",
#     header="time,theta,theta_dot",
# )

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
