#  Simulator #

The simulator class can simulated and animate the pendulum motion forward in time.

## Requirements #


## API #

The simulator should by initialized with a plant (here the PendulumPlant) as follows

    pendulum = PendulumPlant()
    sim = Simulator(plant=pendulum)

To simulate the dynamics of the plant forward in time call

    T, X, TAU = sim.simulate(t0=0.0,
                             x0=[0.5, 0.0],
                             tf=10.0,
                             dt=0.01,
                             controller=None,
                             integrator="runge_kutta")

The inputs of the funciton are

- t0: float, start time, unit: s
- x0: start state (dimension as the plant expects it)
- tf: float, final time, unit: s
- dt: float, time step, unit: s
- controller: controller that computes the motor torque(s) to be applied. The controller should have the structure of the AbstractController class in utilities/abstract_controller. If controller=None, no controller is used and the free system is simulated.
- integrator: string, "euler" for euler integrator,
                      "runge_kutta" for Runge-Kutta integrator

The function returns three lists

- T: List of time values
- X: List of states
- TAU: List of actuations

The same simulation can be executed together with an animation of the plant (only implemented for 2d serial chains). For the simuation with animation call:

    T, X, TAU = sim.simulate_and_animate(t0=0.0,
                                         x0=[0.5, 0.0],
                                         tf=10.0,
                                         dt=0.01,
                                         controller=None,
                                         integrator="runge_kutta",
                                         phase_plot=True,
                                         save_video=False,
                                         video_name="")

The additional parameters are:

- phase plot: bool, whether to show a phase plot along the animation plot
- save_video: bool, whether to save the animation as mp4 video
- video_name: string, name of the file where the video should be saved (only used if save_video=True)



## Usage #

For examples of usages of the simulator class check out the simulation scripts in the controller subfolders (i.e. controllers/energy_shaping/sim_energy_shaping.py)

## Comments #

