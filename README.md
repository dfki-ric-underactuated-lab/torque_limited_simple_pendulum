<div align="center">

#  Torque Limited Simple Pendulum
</div>

<div align="center">
<img width="400" src="../hardware/simple_pendulum_CAD.png">
<img width="300" src="../docs/pendulum_light_painting.jpg">
</div>

## Introduction #
The project is an open-source and low-cost kit to get started with underactuated robotics. The kit targets lowering the entry barrier for studying underactuation in real systems which is often overlooked in conventional robotics courses. It implements a **torque-limited simple pendulum** built using a quasi-direct drive motor which allows for a low friction, torque limited setup. This project describes the _offline_ and _online_ control methods which can be studied using the kit, lists its components, discusses best practices for implementation, presents results from experiments with the simulator and the real system. This repository describes the hardware (CAD, Bill Of Materials (BOM) etc.) required to build the physical system and provides the software (URDF models, simulation and controller) to control it.

## Overview of Methods #

**Trajectory Optimization**
* [Direct Collocation](software/python/simple_pendulum/trajectory_optimization/direct_collocation): offline computed trajectory, optimal
* [Iterative Linear Quadratic Regulator (iLQR)](software/python/simple_pendulum/trajectory_optimization/ilqr/README.md): offline computed trajectory, optimal
* [Feasability driven Differential Dynamic Programming (FDDP)](software/python/simple_pendulum/trajectory_optimization/ddp/README.md): offline computed trajectory, optimal

**Open Loop**
* Proportional-Derivative (PD): precomputed trajectory, not optimal
* Proportional-Derivative + Feed-forward Torque Controller: precomputed trajectory, not optimal
* Feed-forward torque Controller: precomputed trajectory, not optimal

**Closed Loop**
* [Linear Quadratic Regulator (LQR)](software/python/simple_pendulum/controllers/lqr/README.md): stabilization only, optimal
* [Energy Shaping](software/python/simple_pendulum/controllers/energy_shaping/README.md): swingup only, not optimal
* [Iterative Linear Quadratic Regulator (iLQR)](software/python/simple_pendulum/controllers/ilqr/README.md): online computed trajectory, optimal, model predictive controller
* [Soft Actor Critic (SAC)](software/python/simple_pendulum/controllers/sac/README.md): offline trained model, optimal, reinforcement learning
* [Deep Deterministic Policy Gradient (DDPG)](software/python/simple_pendulum/controllers/ddpg/README.md): offline trained model, optimal, reinforcement learning

**NOTE:** The controllers are considered optimal if a cost function in terms of the pendulum states and control inputs can be defined and the controller is able to find an optimal solution for that cost function.

**See a video the simple pendulum in action:** [torque limited swing up (UK video still has to be linked here)](/docs/simple_pendulum_swingup.mp4)

<div align="center">
<img width="500" src="../docs/overview.png">
</div>

## Documentation

* [Simple Pendulum Equations](docs/reference.md)
* [Hardware & Testbench Description](docs/testbench_setup.md)
* [Installation Guide](docs/getting_started.md)
* [Usage Instructions](docs/usage.md)
* [Code Testing](docs/code_test.md)

<!---
## Folder Structure #

<table>
	<tr>
        <td><ul>
                <li><b>data/</b>
                <ul>
                    <li><b>models/</b> (Machine Learning models)</li>
                    <li><b>parameters/</b></li>
                    <li><b>trajectories/</b> (CSV files with position, velocity and torque data)</li>
                    <li><b>urdf/</b></li>
                </ul>
                <li><b>docs/</b> (Documentation)
                <li><b>hardware/</b>
                <ul>
                    <li><b>CAD/</b></li>
                </ul>
                <li><b>results/</b></li>
                <li><b>software/</b>
                <ul>
                    <li><b>python/</b>
                    <ul>
                        <li><b>controllers/</b></li>
                        <li><b>filters/</b></li>
                        <li><b>model/</b></li>
                        <li><b>simulation/</b></li>
                        <li><b>trajectory_optimization/</b></li>
                        <li><b>utilities/</b></li>
                </ul>
                </ul>
            </ul>
    </tr>
</table>

-->

## Safety Notes #

* Brushless motors can be very powerful, moving with tremendous force and speed. Always limit the range of motion, power, force and speed using configurable parameters, current limited supplies, and mechanical design.

* Stay away from the plane in which pendulum is swinging. It is recommended to have a safety net surrounding the pendulum in case the pendulum flies away.

* Make sure you have access to emergency stop while doing experiments. Be extra careful while operating in pure torque control loop. 

## Authors #

<img align="right" img width="500" src="../docs/pendulum_swingup_animation.gif" />

* [Shivesh Kumar](https://robotik.dfki-bremen.de/en/about-us/staff/shku02.html) (Project Supervisor)
* [Felix Wiebe](https://robotik.dfki-bremen.de/en/about-us/staff/fewi01.html) (Software Maintainer)
* [Jonathan Babel](https://robotik.dfki-bremen.de/en/about-us/staff/joba02.html) (Hardware Maintainer)
* [Daniel Harnack](https://robotik.dfki-bremen.de/en/about-us/staff/daha03.html)
* [Heiner Peters](https://robotik.dfki-bremen.de/en/about-us/staff/hepe02.html)
* [Shubham Vyas](https://robotik.dfki-bremen.de/en/about-us/staff/shvy01/)
* [Melya Boukheddimi](https://robotik.dfki-bremen.de/en/about-us/staff/mebo01/)
* [Mihaela Popescu](https://robotik.dfki-bremen.de/en/about-us/staff/mipo02/)

Feel free to contact us if you have questions about the test bench. Enjoy!


## Contributing

1. Fork it (<https://github.com/yourname/yourproject/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

## Acknowledgements #
This work has been performed in the VeryHuman project funded by the German Aerospace Center (DLR) with federal funds (Grant Number: FKZ 01IW20004) from the Federal Ministry of Education and Research (BMBF) and is additionally supported with project funds from the federal state of Bremen for setting up the Underactuated Robotics Lab (Grant Number: 201-001-10-3/2021-3-2).

## License

This work has been released under the BSD 3-Clause License. Details and terms of use are specified in the LICENSE file within this repository. Note that we do not publish third-party software, hence software packages from other developers are released under their very own terms and conditions, e.g. Stable baselines (MIT License) and Tensorflow (Apache License v2.0). If you install third-party software packages along with this repo ensure  that you follow each individual license agreement.   

-----------------------------------------------------------------------------------------------------------------------------
