#  Examples (simulation)

This folder contains example scripts for the implemented trajectory optimization, reinforcement learning and control methods.

## Trajectory Optimization

### Differential Dynamic Programming (DDP)

Trajectory optimization with DDP is explained [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/trajectory_optimization/ddp).

It can be tested with

    python compute_BOXFDDP.py

The script computes a trajectory and stores the trajectory in a csv file at "log_data/ddp/trajectory.csv". The trajectory is also simulated as open loop controller with pinocchio and visualized with the gepetto viewer.

Requirements: Crocoddyl

### Direct Collocation

Trajectory optimization with direct collocation is explained [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/trajectory_optimization/direct_collocation). 

It can be tested with

    python compute_dircol_swingup.py

The script computes a trajectory and stores the trajectory in a csv file at "log_data/direct_collocation/trajectory.csv". The trajectory is also simulated with a [TVLQR](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/controllers/tvlqr) controller which stabilizes the trajectory.

Requirements: Drake

### Iterative Linear Quadratic Regulator (iLQR)

Trajectory optimization with iLQR is explained [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/trajectory_optimization/ilqr).

It can be tested with

    python compute_iLQR_swingup.py

The script computes a trajectory and stores the trajectory in a csv file at "log_data/ilqr/trajectory.csv". The trajectory is also simulated as open loop controller.

Requirements: Drake (optional)

If Drake is installed the symbolic libary of Drake will be used, else sympy is used.

## Reinforcement Learning

### State Actor Critic (SAC)

The SAC reinforcement learning method is explained [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/controllers/sac).

To train a model:

    python train_sac.py

This might take a while. The model will be saved at "log_data/sac_training/sac_model.zip". A trained model is provided [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/data/models/sac_model.zip). It can be tested with

    python sim_sac.py

This will simulate and visualize the behavior of the pendulum following this policy for 10s.

Requirements: torch, stable-baselines3

Note: The provided model was trained with versions: torch==1.10.0 and stable-baselines==1.3.0. Loading it with other versions may cause errors.

### Deep Deterministic Policy Gradient (DDPG)

The DDPG reinforcement learning method is explained [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/controllers/ddpg).

To train a model:

    python train_ddpg.py

This might take a while. The model will be saved at "log_data/ddpg_training/ddpg_model". A trained model is provided [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/data/models/ddpg_model). It can be tested with

    python sim_ddpg.py

This will simulate and visualize the behavior of the pendulum following this policy for 10s.

Requirements: tensorflow

Note: The provided model was trained with versions: tensorflow==2.6.1. Loading it with other versions may cause errors.

## Closed Loop Control

### Linear Quadratic Regulator (LQR)

The LQR method is explained [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/controllers/lqr).

It can be tested with

    python sim_lqr.py

This will simulate and visualize the pendulum with control inputs from the LQR controller.
The start state of the pendulum is near the top position within the region of attraction of the LQR controller. The LQR controller stabilizes the pendulum at the top position.

### Energy Shaping

The energy shaping method is explained [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/controllers/energy_shaping).

It can be tested with

    python sim_energy_shaping.py

This will simulate and visualize the pendulum with control inputs from the energy shaping controller. Near the top position the controller will switch to the LQR controller to stabilize the pendulum.´

### Model Predictive Control (MPC) with iLQR

Model Predictive control with iLQR is explained [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/controllers/ilqr).

It can be tested with

    python sim_ilqrMPC.py

This will simulate and visualize the pendulum with control inputs from the ilqr-MPC controller.
The controller computes an initial guess for a swingup trajectory and then swings up the pendulum and stabilizes the pendulum at the top position.

Requirements: Drake (optional)

If Drake is installed the symbolic libary of Drake will be used, else sympy is used.

## Analysis

### Plotting

The policies of the energy shaping, LQR, SAC and DDPG controllers can be plotted with

    python plot_controller.py

The controller to be plotted can be specified in the script.

### Benchmarking

The controllers can be benchmarked with

    python benchmark_controller.py

This script will perform the benchmark evaluation for all controllers which are specified within the "cons" list in the script. As default all available controllers are listed here. As many iterations of the controllers are necessary to perform the benchmark analysis, the execution of this script may take a while. The results will be written to "log_data/benchmarks".

The benchmarks can be plotted with

    python plot_benchmarks.py

By default the script will load the precomputed benchmarks from [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/data/benchmarks).



