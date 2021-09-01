---
title: 'Torque-limited simple pendulum: A Python package for getting familiar with control algorithms in robotics'
tags:
  - Python
  - robotics
  - underactuated
  - pendulum
  - torque-limited
authors:
  - name: Felix Wiebe^[co-first author] # note this makes a footnote saying 'co-first author'
    orcid: [ToDO ad orcid]
    affiliation: 1 # (Multiple affiliations must be quoted)
  - name: Jonathan Babel^[co-first author] # note this makes a footnote saying 'co-first author'
    affiliation: 1
  - name: Shubham Vyas
    affiliation: 1
  - name: Daniel Harnack
    affiliation: 1
  - name: Melya Boukheddimi
    affiliation: 1
  - name: Shivesh Kumar
    affiliation: 1
affiliations:
 - name: DFKI GmbH Robotics Innovation Center, Bremen, Germany
   index: 1

date: 31 August 2021
bibliography: paper.bib
---

# Summary

There are many, wildly different approaches to robotic control. 
Underactuated robots are systems for which it is not possible to dicatate arbitrary accelerations to all joints. Hence, a controller cannot be used to overwrite the system dynamics and force the system on a desired trajectory as it often is done in classical control techniques.
A simple torque-limited is arguably the simplest, underactuated robotic system and can be used to study, test and benchmark different controllers.

This repository describes the hardware (CAD, Bill Of Materials (BOM) etc.) required to build the
physical pendulum system and provides the software (URDF models, simulation and controller) to control it. It provides a setup for studying established and novel control methods on a simple torque-limited pendulum, and targets students and beginners of robotic control.


# Statement of need

This repository is designed to be used in education and research. It targets lowering the entry barrier 
for studying underactuation in real systems which is often overlooked in conventional robotics courses.
With this software package, students who want to learn about robotics, optimal control or reinforcement learning can make hands-on
experiences with hardware and software for robot control. 
This dualistic approach of describing software and hardware is chosen to motivate experiments with real robotic hardware and facilitate the transfer between software and hardware.
To ensure reproducibility and evaluate novel control methods, not just the control methods' code but also results from experiments are available.


# Background
## System Design

...

## Pendulum Dynamics

![Simple Pendulum.](figures/simple_pendulum_CAD.png){#id .class height=400px}

The equations of motion of a pendulum are

$$I\ddot{\theta} + b\dot{\theta} + c_f \text{sign}(\dot{\theta}) + mgl \sin(\theta) = \tau$$


where

- $\theta$, $\dot{\theta}$, $\ddot{\theta}$ are the angular displacement, angular velocity and angular acceleration of the pendulum. $\theta=0$ means the pendulum is at its stable fixpoint (i.e. hanging down).
- $I$ is the inertia of the pendulum. For a point mass: $I=ml^2$
- $m$ mass of the pendulum
- $l$ length of the pendulum
- $b$ damping friction coefficient
- $c_f$ coulomb friction coefficient
- $g$ gravity (positive direction points down)
- $\tau$ torque applied by the motor

This project provides an easy accessible plant for the pendulum dynamics which is built up from scratch and uses only standard libraries. The plant can be passed to a simulator object, which is capable of integrating the equations of motion und thus simulating the time evolution of the pendulum. The simulator can do Euler and Runge-Kutta integration and can also visualize the motion in a matplotlib animation. Furthermore, it is possible to interface a controller to the simulator which sends control a signal in form of a torque $\tau$ to the motor.

The pendulum has two fixpoints, one of them being stable (the pendulum hanging down) and the other being unstable (the pendulum pointing upwards). A challenge from the control point of view is to swing up the pendulum to the unstable fixpoint and stabilize the pendulum in that state.

## Control Methods

The challenge that serves as a benchmark for various control algorithms is to swing-up the pendulum, if the torque $\tau$ that the motor can output is limited. If the torque limit is set low enough, the pendulum is no longer able to simply go up to the unstable fixpoint but instead the pendulum has to swing and built up energy in the system.

The control methods that are currently implemented in this library can be grouped in three categories:

**Trajectory optimization** tries to find a trajectory of control inputs and states that is feasible for the system while minimizing a cost funtion. The cost function can for example include terms which drive the system to a desired goal state and penalize the usage of high torques. The following trajectory optimization algorithms are implemented:

- Direct Collocation [@hargraves1987direct]
- Iterative Linear Quadratic Regulator (iLQR) [@weiwei2004iterative]
- Feasibility driven Differential Dynamic Programming (FDDP) [@mastalli2020crocoddyl]

**Closed Loop Controllers** or feedback controllers take the state of the system as input and ouput a control signal. Because they are able to react to the current state, they can cope with perturbations during the execution. The following feedback controllers are implemented:

- Energy Shaping
- Linear Quadratic Regulator (LQR)
- Gravity Compensation
- Model Predictive Control (MPC) with iLQR

**Reinforcement Learning** (RL) can be used to learn a policy on the state space of the robot. The policy, which has to be trained beforehand, receives a state and outputs a control signal like a feedback controller. The simple pendulum is can be formulated as a RL problem with two continuous inputs and one continuous output. Similar to the cost function in trajectory optimization, the policy is trained with a reward function. The following RL algorithms are implemented:

- Soft Actor Critic (SAC) [@haarnoja2018soft]
- Deep Deterministic Policy Gradient (DDPG) [@lillicrap2019continuous]


## Tools

To get an understanding of the functionality of the implemented controllers they can be visualized in the pendulum's state space. Example visualizations of the energy shaping controller and the policy learned with DDPG are shown in figure \autoref{fig:controller_plots}.

![Energy Shaping Controller and DDPG Policy. \label{fig:controller_plots}](figures/energy_and_ddpg.png){#id .class height=600px}

Furthermore, the swing-up controllers can be benchmarked, where it is evaluated how fast, efficient, consistent, stable and sensitive the controller is during the swingup. See figure \autoref{fig:benchmark} for the examplary results of the energy shaping controller.

![Energy Shaping Benchmark results. \label{fig:benchmark}](figures/benchmark.png){#id .class height=1200px}

<!--
# Citations

Citations to entries in paper.bib should be in
[rMarkdown](http://rmarkdown.rstudio.com/authoring_bibliographies_and_citations.html)
format.

If you want to cite a software repository URL (e.g. something on GitHub without a preferred
citation) then you can do it with the example BibTeX entry below for @fidgit.

For a quick reference, the following citation commands can be used:
- `@author:2001`  ->  "Author et al. (2001)"
- `[@author:2001]` -> "(Author et al., 2001)"
- `[@author1:2001; @author2:2001]` -> "(Author1 et al., 2001; Author2 et al., 2002)"

# Figures

Figures can be included like this:
![Caption for example figure.\label{fig:example}](figure.png)
and referenced from text using \autoref{fig:example}.

Figure sizes can be customized by adding an optional second parameter:
![Caption for example figure.](figure.png){ width=20% }
-->

# Acknowledgements

# References
