# Soft Actor Critic Training

Type: Closed loop, learning based, model free

State/action space constraints: None

Optimal: Yes

Versatility: Swing-up and stabilization

## Theory

The soft actor critic (SAC) algorithm is a reinforcement learning (RL) 
method. It belongs to the class of so called 'model free' 
methods, i.e. no knowledge about the system to be controlled is 
assumed. Instead, the controller is trained via interaction with 
the system, such that a (sub-)optimal mapping from state space 
to control command is learned. The learning process is guided by 
a reward function that encodes the task, similar to the usage of 
cost functions in optimal control. 

SAC has two defining features. 
Firstly, the mapping from state space to control command is probabilistic. 
Secondly, the entropy of the contol output is maximized along with the reward 
function during training.
In theory, this leads to robust controllers and reduces the probability of 
ending up in suboptimal local minima.

For more information on SAC please refer to the original paper [[1]](https://arxiv.org/abs/1801.01290):


## API

The sac trainer can be initialized with

    trainer = sac_trainer(log_dir="log_data/sac_training")

During and after the training process the trainer will save logging data as well the best model in the directory provided via the log_dir parameter.

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

The parameters of init_agent are:

- learning_rate: learning_rate of the agent
- warm_start: whether to warm_start the agent
- warm_start_path: path to a model to load for warm starting
- verbose: Whether to print training information to the terminal

After these initialisations the training can be started with

    trainer.train(training_timesteps=1e6,
                  reward_threshold=1000,
                  eval_frequency=10000,
                  n_eval_episodes=20,
                  verbose=1)

where the parameters are:

- training_timesteps: Number of timesteps to train
- reward_threshold: Validation threshold to stop training early
- eval_frequency: evaluate the model every eval_frequency timesteps
- n_eval_episodes: number of evaluation episodes
- verbose: Whether to print training information to the terminal

When finished the train method will save the best model in the log_dir of the trainer object.

:warning: **Attention**: when training is started, 
the log_dir will be deleted. So if you want to keep a trained 
model, move the saved files somewhere else.

The training progress can be observed with tensorboard. Start a new terminal and start the tensorboard with the correct path, e.g.:

    $> tensorboard --logdir log_data/sac_training/tb_logs

The default reward function used during training is "soft_binary_with_repellor"
```math
\begin{equation}
r =  \exp{-(\theta - \pi)^2/(2*0.25^2)} - \exp{-(\theta - 0)^2/(2*0.25^2)}
\end{equation}
```
This encourages moving away from the stable fixed point of the system 
at <img src="https://render.githubusercontent.com/render/math?math=\theta = 0"> and spending most time at the target, the unstable 
fixed point <img src="https://render.githubusercontent.com/render/math?math=\theta = \pi">. Different reward functions can be used. 
Novel reward funcitons can be implemented by modifying the *swingup_reward* method of the training environment with 
an appropriate *if* clause, and then selecting this reward function in 
the init_environment parameters under the key *'reward_type'*. The training 
enviroment is located in [gym_environment](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/simulation/gym_environment.py)


## Usage #

For an example of how to train a sac model see the [train_sac.py](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/examples/train_sac.py) script in the examples folder.

The trained model can be used with the [sac controller](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/controllers/sac).

## Comments #

Todo: comments on training convergence stability


## Requirements #

- Stable Baselines 3 (https://github.com/DLR-RM/stable-baselines3)
- Numpy
- PyYaml

## References

[1] [Haarnoja, Tuomas, et al. "Soft actor-critic: Off-policy maximum entropy deep reinforcement learning with a stochastic actor." International conference on machine learning. PMLR, 2018.](https://arxiv.org/abs/1801.01290)




