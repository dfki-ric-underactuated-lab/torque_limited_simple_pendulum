#  Simulator #

The simulator class can simulate and animate the pendulum motion forward in time.
The gym environment can be used for reinforcement learning.

## API #

### The simulator #

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

The inputs of the function are

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


### The gym environment #

The environment can be initialized with

    pendulum = PendulumPlant()
    sim = Simulator(plant=pendulum)
    env = SimplePendulumEnv(simulator=sim,
                            max_steps=5000,
                            target=[np.pi, 0.0],
                            state_target_epsilon=[1e-2, 1e-2],
                            reward_type='continuous',
                            dt=1e-3,
                            integrator='runge_kutta',
                            state_representation=2,
                            validation_limit=-150,
                            scale_action=True,
                            random_init="False")

The parameters are:

- simulator : simulator object
- max_steps : int, default=5000, maximum steps the agent can take before the episode is terminated
- target : array-like, default=[np.pi, 0.0], the target state of the pendulum
- state_target_epsilon: array-like, default=[1e-2, 1e-2], target epsilon for discrete reward type
- reward_type : string, default='continuous', the reward type selects the reward function which is used
             options: 'continuous', 'discrete', 'soft_binary',
                          'soft_binary_with_repellor'
- dt : float, default=1e-3, timestep for the simulation
- integrator : string, default='runge_kutta', the integrator which is used by the simulator
             options : 'euler', 'runge_kutta'
- state_representation : int, default=2, determines how the state space of the pendulum is represented
             2 means state = [position, velocity]
             3 means state = [cos(position), sin(position), velocity]
- validation_limit : float, default=-150, If the reward during validation episodes surpasses this value
             the training stops early
- scale_action : bool, default=True, whether to scale the output of the model with the torque limit
             of the simulator's plant. If True the model is expected so return values in the intervall [-1, 1] as action.
- random_init : string, default="False",
             A string determining the random state initialisation
             "False" : The pendulum is set to [0, 0],
             "start_vicinity" : The pendulum position and velocity
                                are set in the range [-0.31, -0.31],
             "everywhere" : The pendulum is set to a random state in the whole
                            possible state space

## Usage #

For examples of usages of the simulator class check out the scripts in the [examples folder](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/examples).

The gym environment is used for example in the [ddpg training](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/reinforcement_learning/ddpg).

## Comments #

