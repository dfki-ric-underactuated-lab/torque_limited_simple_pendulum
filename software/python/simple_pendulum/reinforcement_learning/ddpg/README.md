# Deep Deterministic Policy Gradient Training #

Type: Closed loop, learning based, model free

State/action space constraints: None

Optimal: Yes

Versatility: Swing-up and stabilization

## Theory

The Deep deterministic Policy Gradient (DDPG) algorithm is a reinforcement learning (RL) method. DDPG is a model-free off-policy algorithm.  The controller is trained via interaction with 
the system, such that a (sub-)optimal mapping from state space 
to control command is learned. The learning process is guided by 
a reward function that encodes the task, similar to the usage of 
cost functions in optimal control. 

DDPG can thought of Q-learning for continuous action spaces. 
It utilizes two networks, an actor and a critic. The actor receives a state and proposes an action. The critic assigns a value to a state action pair. The better an action suits a state the higher the value.

Further, DDPG makes use of target networks in order to stabilize the training. This means there are a training and a target version of the actor and critic models. The training version is used during training and the target networks are partially updated by polyak averaging:

<img src="https://render.githubusercontent.com/render/math?math=\phi_{targ} = \tau \phi_{targ} %2B (1 - \tau) \phi_{train}">

where <img src="https://render.githubusercontent.com/render/math?math=\tau"> is usually small.

DDPG also makes use of a replay buffer, which is a set of experiences which have been observed during training. The replay buffer should be large enough to contain a wide range of experiences.
For more information on DDPG please refer to the original paper [[1]](https://arxiv.org/abs/1509.02971v6):

This implementation losely follows the [keras guide [2]](https://keras.io/examples/rl/ddpg_pendulum/).


## API

The ddpg trainer can be initialized with

    trainer = ddpg_trainer(batch_size=64,
                           validate_every=20,
                           validation_reps=10,
                           train_every_steps=1)

where the parameters are

- batch_size: number of samples to train on in one training step
- validate_every: evaluate the training progress every validate_every episodes
- validation_reps: number of episodes used during the evaluation
- train_every_steps: frequency of training compared to taking steps in the environment

Before the training can be started the following three initialisations have to be made:

    trainer.init_pendulum()
    trainer.init_environment()
    trainer.init_agent()

:warning: **Attention** Make sure to call these functions in this order as they build upon another.

The parameters of init_pendulum are:

- mass: mass of the pendulum
- length: length of the pendulum
- inertia: inertia of the pendulum
- damping: damping of the pendulum
- coulomb_friction: coulomb_friction of the pendulum
- gravity: gravity
- torque_limit: torque limit of the pendulum

The parameters of init_environment are:

- dt: timestep in seconds
- integrator: which integrator to use ("euler" or "runge_kutta")
- max_steps: maximum number of timesteps the agent can take in one episode
- reward_type: Type of reward to use receive from the environment ("continuous", "discrete", "soft_binary", soft_binary_with_repellor" and "open_ai_gym")
- target: the target state ([np.pi, 0] for swingup)
- state_target_epsilon: the region around the target which is considered as target
- random_init: How the pendulum is initialized in the beginning of each episode ("False", "start_vicinity", "everywhere")
- state_representation: How to represent the state of the pendulum (should be 2 or 3). 2 for regular representation (position, velocity). 3 for trigonometric representation (cos(position), sin(position), velocity).
- validation_limit: Validation threshold to stop training early

The parameters of init_agent are:

- replay_buffer_size: The size of the replay_buffer
- actor: the tensorflow model to use as actor during training. If None, the default model is used
- critic: the tensorflow model to use as actor during training. If None, the default model is used
- discount: The discount factor to propagate the reward during one episode
- actor_lr: learning rate for the actor model
- critic_lr: learning rate for the critic model
- tau: determines how much of the training models is copied to the target models

After these initialisations the training can be started with

    trainer.train(n_episodes=1000,
                  verbose=True)

where the parameters are:

- n_episodes: number of episodes to train
- verbose: Whether to print training information to the terminal

Afterwards the trained model can be saved with

    trainer.save("log_data/ddpg_training")

The save method will save the actor and critic model under the given path.


:warning: **Attention**: This will delete existiong models at this location. So if you want to keep a trained 
model, move the saved files somewhere else.

The default reward function used during training is "open_ai_gym"

<img src="https://render.githubusercontent.com/render/math?math=r=-(\theta-\pi)^{2}-0.1 (\dot{\theta}-0)^{2}-0.001 u^{2}">

This encourages spending most time at the target (in the equation ([pi, 0]) with actions as small as possible. Different reward functions can be used by changing the reward type in init_environment. 
Novel reward functions can be implemented by modifying the *swingup_reward* method of the training environment with 
an appropriate *if* clause, and then selecting this reward function in 
the init_environment parameters under the key *'reward_type'*. The training 
enviroment is located in [gym_environment](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/simulation)

## Usage

For an example of how to train a sac model see the [train_ddpg.py](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/examples/sim_ddpg.py) script in the examples folder.

The trained model can be used with the [ddpg controller](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/controllers/ddpg).

## Comments
Todo: comments on training convergence stability


## Requirements

- Tensorflow 2.x

## References

[1] Lillicrap, Timothy P., et al. "Continuous control with deep reinforcement learning." arXiv preprint [arXiv:1509.02971 (2015).](https://arxiv.org/abs/1509.02971v6)

[2] [keras guide](https://keras.io/examples/rl/ddpg_pendulum/)


