import numpy as np

from simple_pendulum.reinforcement_learning.sac.sac import sac_trainer


# pendulum parameters
torque_limit = 1.5
mass = 0.57288
length = 0.5
damping = 0.15
gravity = 9.81
coulomb_fric = 0.0
inertia = mass*length**2

# environment parameters
dt = 0.01
integrator = "runge_kutta"
max_steps = 1000
reward_type = "soft_binary_with_repellor"
target = [np.pi, 0]
target_epsilon = [0.05, 0.05]

# training parameters
learning_rate = 0.0003
training_timesteps = 1e6
reward_threshold = 1000
eval_frequency = 10000
n_eval_episodes = 20
random_init = "start_vicinity"

trainer = sac_trainer(log_dir="log_data/sac_training2")

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
                         target=target,
                         state_target_epsilon=target_epsilon,
                         random_init=random_init,
                         state_representation=3)

trainer.init_agent(learning_rate=learning_rate,
                   warm_start=False,
                   warm_start_path="",
                   verbose=1)

trainer.train(training_timesteps=training_timesteps,
              reward_threshold=reward_threshold,
              eval_frequency=eval_frequency,
              n_eval_episodes=n_eval_episodes,
              verbose=1)
