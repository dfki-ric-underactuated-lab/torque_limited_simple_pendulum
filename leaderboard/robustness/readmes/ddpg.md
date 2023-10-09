# Deep Deterministic Policy Gradient Training

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

This implementation losely follows the [keras guide](https://keras.io/examples/rl/ddpg_pendulum/).

