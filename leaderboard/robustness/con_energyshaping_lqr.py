import numpy as np

# Local imports
from simple_pendulum.controllers.energy_shaping.energy_shaping_controller import (
    EnergyShapingAndLQRController,
)

from sim_parameters import (
    mass,
    length,
    damping,
    gravity,
    coulomb_fric,
    torque_limit,
    inertia,
    dt,
    t_final,
    t0,
    x0,
    goal,
    integrator,
)

name = "energyshaping_lqr"
leaderboard_config = {
    "csv_path": name + "/sim_swingup.csv",
    "name": name,
    "simple_name": "Energy Shaping and LQR",
    "short_description": "Energy shaping for swingup and LQR for stabilization.",
    "readme_path": f"readmes/{name}.md",
    "username": "fwiebe",
}

torque_limit = 1.0

controller = EnergyShapingAndLQRController(
    mass=mass,
    length=length,
    damping=damping,
    coulomb_fric=coulomb_fric,
    gravity=gravity,
    torque_limit=torque_limit,
    k=1.0,
    Q=np.diag([10, 1]),
    R=np.array([[1]]),
    compute_RoA=False,
)
controller.set_goal(goal)
