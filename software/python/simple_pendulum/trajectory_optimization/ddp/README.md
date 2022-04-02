# SwingUp the Simple Pendulum using direct Optimal Control based on the FDDP algorithm:

In this package, the single pendulum swing-up is performed using the direct optimal control based on the FDDP algorithm [(C. Mastalli, 2019)](https://arxiv.org/abs/1909.04947).

The script uses FDDP, BOXFddp can also be used with the same weights.
BOXFddp allows to enforce the system's torque limits.

The urdf model is modified to fit a pinocchio model.

# Theory

The costs functions for the **Running model** is written as :

<img src="https://render.githubusercontent.com/render/math?math=l = {\sum}_{n=1}^{T-1} \alpha_n \Phi_n(q,\dot{q},\Tau),">

With the following costs and weights, <img src="https://render.githubusercontent.com/render/math?math=t_S"> denoting the final time horizon.

1. _**Torque minimization**_: Minimization of the joint torques for realistic dynamic motions.

    <img src="https://render.githubusercontent.com/render/math?math=\Phi_{1} =  \parallel \Tau (t) \parallel ^{2}_2,  \quad \alpha_1 = 1e-4">

2. _**Posture regularization**_:  giving as input only the final reference posture.

    <img src="https://render.githubusercontent.com/render/math?math=\Phi_{2} = \parallel q(t)-q^{ref}(t_{s-1})\parallel ^{2}_2 ,  \quad \alpha_2 = 1e-5">



* The costs functions for the **Terminal model** are applied to only one node (the terminal node) and is written as :


    <img src="https://render.githubusercontent.com/render/math?math=l_T =  \alpha_T \Phi_T(q,\dot{q}),">

With the following cost and weight, <img src="https://render.githubusercontent.com/render/math?math=T = t_{final}"> the final time horizon.

1. _**Posture regularization**_: giving as input only the final reference posture.


    <img src="https://render.githubusercontent.com/render/math?math=\Phi_{3} = \parallel q(T)-q^{ref}(T)\parallel^{2}_{2}, \quad  \alpha_{3} = 10^{10}">




The weights <img src="https://render.githubusercontent.com/render/math?math=\alpha_i"> for this optimization problem are determined experimentally.

# API

The BOXFDDP algorithm can be executed with the boxfddp_calculator class. It can be initialized with

     ddp = boxfddp_calculator(urdf_path=urdf_path,
                             enable_gui=True,
                             log_dir="log_data/ddp")

The urdf_path should point to the urdf of the pendulum in a format that pinochio accepts. The urdf for a simple pendulum can be found in the data folder of this repository: [simplependul_dfki_pino_Modi.urdf](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/data/urdf/simplependul_dfki_pino_Modi.urdf). enable_gui can be set to True if the trajectory shall be visualized with pinocchio after computation. In the log_dir a urdf with modified pendulum parameters will be stored.

To set the correct pendulum parameters in the urdf with:

    ddp.init_pendulum(mass=mass,
                      length=length,
                      inertia=inertia,
                      damping=damping,
                      coulomb_friction=coulomb_fric,
                      torque_limit=torque_limit)

After that the trajectory can be computed with

    T, TH, THD, U = ddp.compute_trajectory(start_state=np.array[0.0, 0.0]),
                                           goal_state=np.array[np.pi, 0.0]),
                                           weights=np.array([1] + [0.1]*1),
                                           dt=4e-2,
                                           T=150,
                                           running_cost_state=1e-5,
                                           running_cost_torque=1e-4,
                                           final_cost_state=1e10)

where weights contains the weights of the terminal and the running cost model. dt is the timestep length, T is the number of timesteps. running_cost_state, running_cost_torque and final_cost_state are the individal cost weights. The method returns the time trajectory T, the positiontrajectory TH, the velocity trajectory THD and the control trajectory U.

The trajectory can be plotted with

    ddp.plot_trajectory()

or simulated with gepetto with (for this enable_gui has to be set to True during the initialization)

    ddp.simulate_trajectory_gepetto()

# Usage

An example script for the pendulum can be found in the [examples](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/examples) directory. It can be started with

    python compute_BOXFDDP_swingup.py


# Dependencies

[Crocoddyl](https://github.com/loco-3d/crocoddyl)

[Pinocchio](https://github.com/stack-of-tasks/pinocchio)

For the display, [Gepetto](https://github.com/Gepetto/gepetto-viewer)

# References

[1] Mastalli, Carlos, et al. "Crocoddyl: An efficient and versatile framework for multi-contact optimal control." 2020 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2020. [arxiv link](https://arxiv.org/abs/1909.04947)
