import os
import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.reinforcement_learning.ddpg.ddpg import ddpg_trainer

save_dir = "log_data/ddpg_training"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# pendulum parameters
mass = 0.57288
length = 0.5
inertia = mass*length**2
damping = 0.15
coulomb_fric = 0.02
gravity = 9.81
torque_limit = 1.5

# environment parameters
dt = 0.01
integrator = "runge_kutta"
max_steps = 1000
reward_type = "open_ai_gym_red_torque"
target = [np.pi, 0]
target_epsilon = [0.05, 0.05]
random_init = "everywhere"

# training parameters
n_episodes = 1000
batch_size = 64
validate_every = 20
validation_reps = 10
validation_limit = -1800
train_every_steps = 1
state_representation = 3
replay_buffer_size = 50000
actor = None  # use default agent
critic = None  # use default critic
discount = 0.99
actor_lr = 0.0005
critic_lr = 0.001
tau = 0.005

trainer = ddpg_trainer(batch_size=batch_size,
                       validate_every=validate_every,
                       validation_reps=validation_reps,
                       train_every_steps=train_every_steps)

trainer.init_pendulum(mass=mass,
                      length=length,
                      inertia=inertia,
                      damping=damping,
                      coulomb_friction=coulomb_fric,
                      gravity=gravity,
                      torque_limit=torque_limit)

trainer.init_environment(dt=dt,
                         integrator=integrator,
                         max_steps=max_steps,
                         reward_type=reward_type,
                         state_representation=state_representation,
                         validation_limit=validation_limit,
                         target=target,
                         state_target_epsilon=target_epsilon,
                         random_init=random_init)

trainer.init_agent(replay_buffer_size=replay_buffer_size,
                   actor=actor,
                   critic=critic,
                   discount=discount,
                   actor_lr=actor_lr,
                   critic_lr=critic_lr,
                   tau=tau)

rewards, actor_losses, critic_losses = trainer.train(n_episodes=n_episodes,
                                                     verbose=True)

trainer.save(save_dir)

# plotting
MA_WINDOW = 10
ma_rewards = np.convolve(rewards, np.ones(MA_WINDOW)/MA_WINDOW, mode='valid')
episodes = np.arange(0, len(rewards))
ma_episodes = np.arange(len(rewards)-len(ma_rewards), len(rewards))

fig, ax = plt.subplots(1, 2, figsize=(12, 6))

# Reward, per episode and moving average
ax[0].plot(episodes, rewards, '-g', label='reward')
ax[0].plot(ma_episodes, ma_rewards, '-b', label='mov. avg.')
ax[0].set_xlabel('Episode')
ax[0].set_ylabel('Reward')
ax[0].legend()

ax[1].plot(actor_losses, '-r', label='actor loss')
ax[1].plot(critic_losses, '-o', label='critic loss')
ax[1].set_xlabel('Episode')
ax[1].set_ylabel('Loss')
ax[1].legend()
plt.show()
