import numpy as np

from simple_pendulum.analysis.benchmark import benchmarker
from simple_pendulum.controllers.energy_shaping.energy_shaping_controller import EnergyShapingController, \
                                                                                 EnergyShapingAndLQRController
from simple_pendulum.controllers.ilqr.iLQR_MPC_controller import iLQRMPCController
from simple_pendulum.controllers.sac.sac_controller import SacController
from simple_pendulum.controllers.ddpg.ddpg_controller import ddpg_controller

con = "energy_shaping"
#con = "ilqr"
#con = "sac"
#con = "ddpg"

mass = 0.57288
length = 0.5
damping = 0.10
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 2.0
inertia = mass*length*length

# simulation parameters
dt = 0.02
max_time = 10.0
integrator = "runge_kutta"
benchmark_iterations = 100

if con == "energy_shaping":
    controller = EnergyShapingAndLQRController(mass,
                                               length,
                                               damping,
                                               gravity,
                                               torque_limit)
    controller.set_goal([np.pi, 0])

if con == "ilqr":
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

if con == "sac":

    model_path = "log_data/sac_training/best_model/best_model.zip"
    params_path = "../../../data/parameters/sp_parameters_sac3.yaml"

    controller = SacController(model_path=model_path,
                               params_path=params_path)

elif con == "ddpg":

    model_path = "../../../data/models/ddpg_model/actor"
    tl = 1.5
    state_rep = 3

    controller = ddpg_controller(model_path=model_path,
                                 torque_limit=tl,
                                 state_representation=state_rep)

ben = benchmarker(dt=dt,
                  max_time=max_time,
                  integrator=integrator,
                  benchmark_iterations=benchmark_iterations)

ben.init_pendulum(mass=mass,
                  length=length,
                  inertia=inertia,
                  damping=damping,
                  coulomb_friction=coulomb_fric,
                  gravity=gravity,
                  torque_limit=torque_limit)

ben.set_controller(controller)

ben.benchmark(check_speed=True,
              check_energy=True,
              check_time=True,
              check_consistency=True,
              check_stability=True,
              check_sensitivity=True,
              check_torque_limit=True)
