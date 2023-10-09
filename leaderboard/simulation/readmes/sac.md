# Soft Actor Critic (SAC)

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

For more information on SAC please refer to the original [paper](https://arxiv.org/abs/1801.01290):
