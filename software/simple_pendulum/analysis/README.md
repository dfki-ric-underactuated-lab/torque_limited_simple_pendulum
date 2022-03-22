# Controller Analysis

## Controller Benchmarking

The controller benchmarking class can benchmark the controllers in simulation with respect to a predefined properties.

### Definitions

The controller benchmark currently computes the following properties

- **Controller Frequency**: How fast can the controller process a pendulum state and return a control output. Measured in calls per second (Hz).
- **Swingup time**: How long does it take for the controller to swing-up the pendulum from the lower fixpoint to the upper fixpoint. Units: Seconds (s)
- **Energy consumption**: How much energy does the controller use during the swingup motion and holding the pendulum stable afterwards. Energy usage is measured by comparing the energy level of the actuated pendulum with a free falling pendulum. Units: Joule (J).
- **Smoothness** measures how much the controller changes the control output during execution. The calculated value is the standard deviation of the differences of all consecutive control signals.
- **Consistency** measures if the controller is able to drive the pendulum to the unstable fixpoint for varying starting positions and velocities. The start position is randomly chosen between [-pi, pi] and the velocity is drawn from the intervall [-3pi/s, 3pi/s].
- **Robustness** tests the controller abilities to recover from perturbations during the swingup motions. The controller is perturbed four times for 1 entire second with a random amount of torque.
- **Sensitivity**: Here the pendulum parameters (mass, length, friction) are modified without using this knowledge in the controller. This tests how sensitive the controller is to the model parameters.
- **Reduced torque limit**: This test checks a list of torque limits to find the minimal torque limit with which the controller is still able to swing-up the pendulum.

### API

To initialize the benchmark class do:

        from simple_pendulum.analysis.benchmark import benchmarker

        ben = benchmarker(dt=dt,
                          max_time=max_time,
                          integrator=integrator,
                          benchmark_iterations=benchmark_iterations)

where dt is the control frequency, max_time the time of a single motion, integrator the integrator to be used ("euler" or "runge_kutta", see [simulator](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/simulation)) and benchmark_iterations is the number iterations that are used to do the benchmark test.

The parameters of the pendulum can be parsed to the benchmark class with

         ben.init_pendulum(mass=mass,
                           length=length,
                           inertia=inertia,
                           damping=damping,
                           coulomb_friction=coulomb_fric,
                           gravity=gravity,
                           torque_limit=torque_limit)

The controller to be tested has to be set by

        ben.set_controller(controller)

where controller is a controller inheriting from the [abstract controller class](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/controllers/abstract_controller.py).

The benmark calculations are then started with

        ben.benchmark(check_speed=True,
                      check_energy=True,
                      check_time=True,
                      check_smoothness=True,
                      check_consistency=True,
                      check_robustness=True,
                      check_sensitivity=True,
                      check_torque_limit=True,
                      save_path="benchmark.yml")

The individual checks can be turned off. The results will be stored in the filed specified in save_path.

### Usage

An example usage can be found in the [examples folder](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/examples) in the [benchmark_controller.py](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/examples/benchmark_controller.py) script.

## Plotting Controllers

Controllers can be plotted by plotting their control signal in the pendulum's state space.

### API

A controller inheriting from the [abstract controller class](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/controllers/abstract_controller.py) can be plotted by using

    from simple_pendulum.analysis.plot_policy import plot_policy

    plot_policy(controller,
                position_range=[-3.14, 3.14],
                velocity_range=[-2. 2],
                samples_per_dim=100,
                plotstyle="3d",
                save_path=None)

The plotstyle can also be set to "2d". If a save_path is specified the plot will be stored in that location.


### Usage

An example usage can be found in the [examples folder](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/examples) in the [plot_controller.py](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/examples/plot_controller.py) script.
