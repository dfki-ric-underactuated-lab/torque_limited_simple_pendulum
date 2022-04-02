import os
import numpy as np

from simple_pendulum.trajectory_optimization.ddp.boxfddp import boxfddp_calculator

log_dir = "log_data/ddp"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# pendulum parameters
mass = 0.57288
length = 0.5
inertia = mass*length*length
damping = 0.10
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 2.0
urdf_path = "../../../data/urdf/simplependul_dfki_pino_Modi.urdf"


# swingup parameters
x0 = [0.0, 0.0]
goal = [np.pi, 0.0]

# ddp parameters
start_state = np.array([0., 0.])
goal_state = np.array([np.pi, 0.])
weights = np.array([1] + [0.1] * 1)
dt = 4e-2
T = 150
running_cost_state = 1e-5
running_cost_torque = 1e-4
final_cost_state = 1e10

ddp = boxfddp_calculator(urdf_path=urdf_path,
                         enable_gui=True,
                         log_dir=log_dir)
ddp.init_pendulum(mass=mass,
                  length=length,
                  inertia=inertia,
                  damping=damping,
                  coulomb_friction=coulomb_fric,
                  torque_limit=torque_limit)

T, TH, THD, U = ddp.compute_trajectory(start_state=start_state,
                                       goal_state=goal_state,
                                       weights=weights,
                                       dt=dt,
                                       T=T,
                                       running_cost_state=running_cost_state,
                                       running_cost_torque=running_cost_torque,
                                       final_cost_state=final_cost_state)
ddp.plot_trajectory()
ddp.simulate_trajectory_gepetto()

# Save Trajectory to a csv file to be sent to the motor.
csv_data = np.vstack((T, TH, THD, U)).T
csv_path = os.path.join(log_dir, "trajectory.csv")
np.savetxt(csv_path, csv_data, delimiter=',',
           header="time,pos,vel,torque", comments="")
